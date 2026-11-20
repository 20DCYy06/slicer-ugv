#!/usr/bin/env python3
"""One serialized computer/aircraft sender for the public air-ground V1 topics.

Each command is published once. A receipt means accepted, never task completed.
Use saved captured target files or supply the real WGS84 base_link destination.
Run one logical producer at a time; do not issue concurrent CLI commands.
"""
import argparse
import json
import math
from pathlib import Path
import sys
import threading
import time
import uuid

sys.path.insert(0, str(Path(__file__).resolve().parent))
from air_ground_wire import (COMMAND_TOPIC, PROTOCOL_VERSION, STATE_TOPIC, UINT64_MAX,
                             load_target, make_command, normalize_command, normalize_point,
                             state_problem)

COMMAND_NAMES = dict(set='SET_TASK', **{'replace-via': 'REPLACE_VIA_POINTS'},
                     pause='PAUSE', resume='RESUME', cancel='CANCEL')


def number(value):
    result = float(value)
    if not math.isfinite(result):
        raise argparse.ArgumentTypeError('must be finite')
    return result


def positive(value):
    result = number(value)
    if result <= 0.:
        raise argparse.ArgumentTypeError('must be greater than zero')
    return result


def arguments(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    common = argparse.ArgumentParser(add_help=False)
    common.add_argument('--state-timeout', type=positive, default=5., help='seconds to obtain fresh state')
    common.add_argument('--state-max-age', type=positive, default=2., help='maximum state receive/stamp age in seconds')
    common.add_argument('--connect-timeout', type=positive, default=3., help='seconds to wait for command subscriber')
    common.add_argument('--receipt-timeout', type=positive, default=3., help='seconds to wait for this request receipt')
    commands = parser.add_subparsers(dest='command', required=True)
    for name in ('set', 'replace-via', 'pause', 'resume', 'cancel', 'status', 'replay'):
        command = commands.add_parser(name, parents=[common])
        if name in COMMAND_NAMES:
            command.add_argument('--save-request', type=Path, help='save exact request before publishing; never overwrites')
        if name in ('set', 'replace-via'):
            command.add_argument('--via-file', type=Path, action='append', default=[], help='ordered captured WGS84 target; repeat per via')
        if name == 'set':
            command.add_argument('--target-file', type=Path, help='existing geographic capture JSON')
            command.add_argument('--latitude', type=number, help='WGS84 degrees')
            command.add_argument('--longitude', type=number, help='WGS84 degrees')
            command.add_argument('--altitude-ellipsoid', type=number, help='optional WGS84 ellipsoid metres')
            command.add_argument('--yaw', type=number, help='optional final yaw, radians CCW from WGS84 ENU East')
        if name == 'replay':
            command.add_argument('request_file', type=Path, help='exact JSON saved by --save-request; same receiver boot only')
    args = parser.parse_args(argv)
    if args.command == 'set':
        raw = any(value is not None for value in (args.latitude, args.longitude, args.altitude_ellipsoid))
        if args.target_file is not None and raw:
            parser.error('--target-file cannot be combined with WGS84 coordinate options')
        if args.target_file is None:
            try:
                normalize_point(dict(latitude=args.latitude, longitude=args.longitude,
                                     altitude_ellipsoid_m=args.altitude_ellipsoid, yaw=args.yaw))
            except ValueError as error:
                parser.error(str(error))
    return args


def build_request(args, state, identifier=None):
    """Make a complete atomic request from fresh state; sender holds no lease."""
    identifier = identifier or (lambda: str(uuid.uuid4()))
    if args.command == 'replay':
        with args.request_file.open(encoding='utf-8') as stream:
            command = normalize_command(json.load(stream))
        if command['server_id'] != state['server_id']:
            raise ValueError('saved request server_id differs; receiver restarted, create a new explicit command')
        return command
    if args.command not in COMMAND_NAMES:
        raise ValueError('status does not create a command')
    sequence = state.get('last_sequence') or 0
    if type(sequence) is not int or not 0 <= sequence < UINT64_MAX:
        raise ValueError('receiver sequence exhausted or invalid; cannot allocate a new sequence')
    final = None
    if args.command == 'set':
        final = (load_target(args.target_file, args.yaw) if args.target_file is not None else
                 normalize_point(dict(latitude=args.latitude, longitude=args.longitude,
                                      altitude_ellipsoid_m=args.altitude_ellipsoid, yaw=args.yaw)))
    via = [load_target(path) for path in getattr(args, 'via_file', [])]
    request_id = identifier()
    task_id = identifier() if args.command == 'set' else state.get('task_id')
    return normalize_command(dict(protocol_version=PROTOCOL_VERSION, server_id=state['server_id'],
                                  sequence=sequence+1, request_id=request_id, task_id=task_id,
                                  operation=COMMAND_NAMES[args.command], final_goal=final, via_points=via))


def save_request(path, command):
    with Path(path).open('x', encoding='utf-8') as stream:
        json.dump(normalize_command(command), stream, indent=2, sort_keys=True, allow_nan=False)
        stream.write('\n')


def matching_receipt(state, request):
    if not isinstance(state, dict) or state.get('server_id') != request['server_id']:
        return None
    receipt = state.get('last_command')
    if (isinstance(receipt, dict) and receipt.get('request_id') == request['request_id']
            and type(receipt.get('accepted')) is bool):
        return receipt
    return None


class AirGroundClient:
    def __init__(self, args):
        import rospy
        from slicer_air_ground_msgs.msg import Command, GeoPoint
        from std_msgs.msg import String
        self.args, self.ros = args, rospy
        self.command_type, self.point_type = Command, GeoPoint
        self.lock = threading.Lock()
        self.state, self.received = None, None
        self.subscriber = rospy.Subscriber(STATE_TOPIC, String, self._state, queue_size=1)
        self.publisher = (None if args.command == 'status' else
                          rospy.Publisher(COMMAND_TOPIC, Command, queue_size=1, latch=False))

    def _state(self, message):
        try:
            value = json.loads(message.data)
        except (ValueError, TypeError):
            value = None
        with self.lock:
            self.state, self.received = value, time.monotonic()

    def observation(self):
        with self.lock:
            return self.state, self.received

    def fresh_state(self):
        deadline = time.monotonic()+self.args.state_timeout
        problem = 'air-ground state unavailable'
        while not self.ros.is_shutdown() and time.monotonic() < deadline:
            state, received = self.observation()
            problem = state_problem(state, received, time.monotonic(), self.ros.Time.now().to_sec(),
                                    max_age=self.args.state_max_age)
            if problem is None:
                return state
            time.sleep(.02)
        raise RuntimeError(problem)

    def publish_once(self, request):
        message = make_command(request, self.command_type, self.point_type)
        deadline = time.monotonic()+self.args.connect_timeout
        while not self.ros.is_shutdown() and self.publisher.get_num_connections() == 0:
            if time.monotonic() >= deadline:
                raise RuntimeError('no command subscriber; request was not published')
            time.sleep(.02)
        if self.ros.is_shutdown():
            raise RuntimeError('ROS shutdown before publishing request')
        self.publisher.publish(message)

    def wait_receipt(self, request):
        deadline = time.monotonic()+self.args.receipt_timeout
        while not self.ros.is_shutdown() and time.monotonic() < deadline:
            state, received = self.observation()
            if state_problem(state, received, time.monotonic(), self.ros.Time.now().to_sec(),
                             max_age=self.args.state_max_age) is None:
                if state['server_id'] != request['server_id']:
                    raise RuntimeError('receiver restarted while awaiting receipt; request outcome unknown')
                receipt = matching_receipt(state, request)
                if receipt is not None:
                    return receipt
            time.sleep(.02)
        raise RuntimeError('receipt timeout; outcome unknown, inspect status or replay the exact saved request')

    def run(self):
        state = self.fresh_state()
        if self.args.command == 'status':
            print(json.dumps(state, indent=2, sort_keys=True, allow_nan=False))
            return 0
        request = build_request(self.args, state)
        if getattr(self.args, 'save_request', None) is not None:
            save_request(self.args.save_request, request)
        self.publish_once(request)
        receipt = self.wait_receipt(request)
        print(json.dumps(dict(request_id=request['request_id'], task_id=request['task_id'],
                              sequence=request['sequence'], receipt=receipt,
                              meaning='accepted is command receipt, not task completion'),
                         indent=2, sort_keys=True, allow_nan=False))
        return 0 if receipt['accepted'] else 1


def main(argv=None):
    args = arguments(argv)
    try:
        import rospy
        rospy.init_node('air_ground_sender', anonymous=True)
        return AirGroundClient(args).run()
    except (ImportError, OSError, ValueError, RuntimeError) as error:
        print('air_ground: ' + str(error), file=sys.stderr)
        return 2


if __name__ == '__main__':
    sys.exit(main())
