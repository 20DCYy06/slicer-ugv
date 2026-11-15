#!/usr/bin/env python3
"""Outdoor task interface. One worker owns move_base; feedback never awaits ACK."""
import json
import hashlib
import queue
import subprocess
import sys
import threading
import time
import uuid
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parent))
from air_ground_core import TaskCore
from air_ground_executor import TaskExecutor
from air_ground_wire import COMMAND_TOPIC, STATE_TOPIC, decode_command
from geographic import GeographicClient
from geographic_goal_core import arrival_review, origin_record, quiet_odom_tail, resolve_target, status_problem


class RosNavigation:
    """Reuse the field-proven converter, observation windows and Mux CLI."""
    def __init__(self, ros, action):
        from std_msgs.msg import String
        from actionlib_msgs.msg import GoalStatusArray
        self.ros = ros
        self.geo = GeographicClient(SimpleNamespace(action=action, command='status', timeout=120.))
        self.mux_sample = None
        self.mux_sub = ros.Subscriber('/cup_nav/manual_state', String, self._mux, queue_size=1)
        self.last_action_state = None
        self.status_sample = None
        self.action_status_sub = ros.Subscriber(action+'/status', GoalStatusArray, self._action_status, queue_size=1)

    def _action_status(self, message):
        if (getattr(message, '_connection_header', {}) or {}).get('callerid') != self.geo.args.action:
            return
        age = (self.ros.Time.now()-message.header.stamp).to_sec()
        if 0 <= age <= .5:
            self.status_sample = time.monotonic(), list(message.status_list)

    def _mux(self, message):
        header = getattr(message, '_connection_header', {}) or {}
        if header.get('callerid') != '/slicer_velocity_mux' or header.get('latching', '0') != '0':
            return
        try:
            value = json.loads(message.data)
            if any(type(value[k]) is not bool for k in ('manual_hold', 'emergency_stop_latched', 'teleop_fresh')):
                return
            if type(value['takeover_generation']) is not int or not isinstance(value['mux_instance_id'], str):
                return
            self.mux_sample = time.monotonic(), value
        except (KeyError, ValueError, TypeError):
            return

    def mux_state(self):
        sample = self.mux_sample
        if sample is None or not 0 <= time.monotonic()-sample[0] <= .5:
            raise ValueError('existing Mux state unavailable/stale')
        return sample[1]

    def status(self, generation=None):
        status, received, _ = self.geo.observation()
        problem = status_problem(status, received, time.monotonic(), self.ros.Time.now().to_sec(),
                                 generation=generation, max_age=self.geo.status_age)
        if problem:
            raise ValueError(problem)
        return status

    def health(self, target=None):
        try:
            self.status(target['generation'] if target else None)
            return None
        except ValueError as error:
            return str(error)

    def _convert(self, point, status, pose):
        height = point.get('altitude_ellipsoid_m')
        if height is None:
            height = origin_record(status)['altitude_ellipsoid_m']
        args = SimpleNamespace(latitude=point['latitude'], longitude=point['longitude'],
                               altitude_ellipsoid=height, yaw=point.get('yaw'), target_file=None,
                               east=None, north=None, up=0., origin_file=None, session_enu=False)
        target = resolve_target(args, status, pose)
        target.update(generation=status['generation'], stamp=self.ros.Time.now().to_sec(),
                      requested_wgs84=dict(point),
                      height_policy=('explicit ellipsoid height; 2D navigation' if point.get('altitude_ellipsoid_m')
                                     is not None else 'task-origin ellipsoid height; 2D Up=0; not measured base height'))
        return target

    def validate_route(self, final, vias):
        self.geo.frame_check('map')
        status, pose = self.status(), self.geo.current_pose('map')
        for point in vias+[final]:
            self._convert(point, status, pose)

    def target(self, point):
        return self._convert(point, self.status(), self.geo.current_pose('map'))

    def position(self):
        return self.geo.current_pose('map')[:2]

    def start(self, target):
        if not self.geo.client.wait_for_server(self.ros.Duration(.2)):
            raise ValueError('existing move_base Action unavailable')
        # Reuse the existing control client's idle-status criterion. Do not
        # preempt a competing legacy goal merely by submitting our first goal.
        deadline = time.monotonic()+1.
        while True:
            sample = self.status_sample
            if (sample is not None and 0 <= time.monotonic()-sample[0] <= .5
                    and all(row.status not in (0, 1, 6, 7) for row in sample[1])):
                break
            if time.monotonic() >= deadline:
                raise ValueError('fresh idle move_base status unavailable; stop other goal senders')
            time.sleep(.02)
        self.geo.client.send_goal(self.geo.MoveBaseGoal(
            target_pose=self.geo.pose(target['map_xyz'], target['yaw'], target['frame'])))

    def poll(self):
        return dict(state=self.geo.client.get_state(), text=self.geo.client.get_goal_status_text(),
                    observed_stamp=self.ros.Time.now().to_sec())

    def cancel(self):
        self.geo.client.cancel_goal()

    def _control(self, operation, expected_hold=None):
        command = ['rosrun', 'ugv_slicer_control', 'control.py', operation]
        if expected_hold is not None:
            command += ['--expected-mux-instance', str(expected_hold[0]),
                        '--expected-generation', str(expected_hold[1])]
        process = subprocess.run(command,
                                 text=True, capture_output=True, timeout=12.)
        for line in reversed(process.stdout.splitlines()):
            try:
                result = json.loads(line)
                if isinstance(result, dict) and 'manual_hold' in result:
                    # control.py prints its confirmed Mux ACK before separately
                    # waiting for cancel-all. Keep that hold ownership even if
                    # the subsequent Action cancel ACK times out. The executor
                    # still waits for its owned Action; this is not a stop ACK.
                    if process.returncode == 0 or (operation == 'hold' and result.get('manual_hold')
                                                   and result.get('last_intent_accepted')):
                        return dict(result, action_cancel_note=process.stderr.strip())
            except ValueError:
                pass
        raise ValueError('existing Mux '+operation+' ACK unavailable: '+process.stderr.strip())

    def hold(self):
        return self._control('hold')

    def release(self, expected_hold):
        return self._control('release', expected_hold)

    def stopped(self, since):
        _, _, rows = self.geo.observation()
        return quiet_odom_tail([r for r in rows if r['received'] >= since], time.monotonic(),
                               self.ros.Time.now().to_sec(), window=self.geo.stop_window)

    def review(self, target, action, since):
        status, received, _ = self.geo.observation()
        return arrival_review(target, status, received, time.monotonic(), self.ros.Time.now().to_sec(),
                              target['generation'], action['state'], self.stopped(since),
                              self.geo.tolerance, self.geo.fix_age, self.geo.status_age)


class TaskNode:
    def __init__(self, ros):
        from slicer_air_ground_msgs.msg import Command
        from std_msgs.msg import String
        self.ros, self.String = ros, String
        self.inbox = queue.Queue(maxsize=32)
        self.latest, self.transport_error = {}, None
        self.last_receipt = None
        self.feedback_lock = threading.Lock()
        self.backend = RosNavigation(ros, ros.get_param('~action', '/ugv/move_base'))
        self.executor = TaskExecutor(TaskCore(uuid.uuid4().hex), self.backend,
                                    via_pass_radius_m=ros.get_param('~via_pass_radius_m', .25),
                                    segment_timeout_s=ros.get_param('~segment_timeout_s', 120.),
                                    cancel_timeout_s=ros.get_param('~cancel_timeout_s', 3.),
                                    stop_timeout_s=max(4., self.backend.geo.stop_window+4.)+self.backend.geo.fix_age)
        directory = Path(__file__).resolve().parent
        self.software = {'files_sha256': {name: hashlib.sha256((directory/name).read_bytes()).hexdigest()
                         for name in ('air_ground_node.py', 'air_ground_core.py', 'air_ground_executor.py',
                                      'air_ground_wire.py', 'geographic.py', 'geographic_goal_core.py')}}
        self.publisher = ros.Publisher(STATE_TOPIC, String, queue_size=1, latch=True)
        self.subscriber = ros.Subscriber(COMMAND_TOPIC, Command, self.receive, queue_size=32)
        self._snapshot()
        self.timer = ros.Timer(ros.Duration(.2), self.publish)
        self.worker = threading.Thread(target=self.run, name='air-ground-action-owner', daemon=True)
        self.worker.start()

    def receive(self, message):
        header = getattr(message, '_connection_header', {}) or {}
        if header.get('latching', '0') != '0':
            self.transport_error = dict(reason='latched commands are refused',
                                        request_id=message.request_id, stamp=self.ros.Time.now().to_sec())
            return
        try:
            command = decode_command(message)
            self.inbox.put_nowait(command)
        except queue.Full:
            self.transport_error = dict(reason='command queue full; retry the identical request',
                                        request_id=message.request_id, stamp=self.ros.Time.now().to_sec())
        except (ValueError, TypeError) as error:
            self.transport_error = dict(reason=str(error), request_id=message.request_id,
                                        stamp=self.ros.Time.now().to_sec())

    def _snapshot(self):
        state = self.executor.snapshot()
        if self.last_receipt is not None:
            state['last_command'] = self.last_receipt
        state.update(execution_updated_stamp=self.ros.Time.now().to_sec(),
                     via_pass_radius_m=self.executor.radius,
                     navigation_action=self.backend.geo.args.action,
                     physical_reference='base_link_horizontal', datum='WGS84',
                     transport_error=self.transport_error, software=self.software)
        with self.feedback_lock:
            self.latest = state

    def publish(self, _event=None):
        with self.feedback_lock:
            value = dict(self.latest)
        value.update(updated_stamp=self.ros.Time.now().to_sec(), transport_error=self.transport_error)
        self.publisher.publish(self.String(data=json.dumps(value, ensure_ascii=False, allow_nan=False)))

    def run(self):
        try:
            while not self.ros.is_shutdown():
                # Bound command work per cycle, retaining navigation/stop polling.
                for _ in range(8):
                    try:
                        command = self.inbox.get_nowait()
                    except queue.Empty:
                        break
                    receipt = self.executor.command(command)
                    self.last_receipt = dict(receipt, request_id=command['request_id'],
                                             sequence=command['sequence'], operation=command['operation'],
                                             task_id=command['task_id'])
                    self.transport_error = None
                    self.ros.loginfo('air_ground receipt: %s', json.dumps(receipt, ensure_ascii=False))
                    self._snapshot()
                self.executor.tick()
                self._snapshot()
                time.sleep(.05)
        except Exception as error:
            self.executor._block('task worker failed: '+str(error))
            self._snapshot()
            self.ros.logerr('air_ground worker failed: %s', error)
            self.ros.signal_shutdown('task worker failed; stop requested')
        finally:
            try:
                self.executor.shutdown()
            except Exception as error:
                self.ros.logerr('air_ground shutdown stop unconfirmed: %s', error)


def main():
    import rospy
    rospy.init_node('air_ground_task')
    node = TaskNode(rospy)
    try:
        rospy.spin()
    finally:
        node.worker.join(timeout=15.)


if __name__ == '__main__':
    main()
