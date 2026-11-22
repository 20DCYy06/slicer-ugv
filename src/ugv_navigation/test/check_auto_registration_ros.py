#!/usr/bin/env python3
"""Exercise one real WGS84 task interface on a private, device-free ROS graph.

Source the task's Noetic install, then run this script for v1 and v2. The
existing differential plant consumes only final Mux Twist. Scan, local odom,
receiver FIXED and chassis-ready are synthetic; navigation and global EKF are
real. The script launches no hardware or transport processes.
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import tempfile
import time
from xmlrpc.client import ServerProxy


def wait_until(check, label, timeout=15., diagnostic=lambda: None):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        result = check()
        if result:
            return result
        time.sleep(.025)
    raise AssertionError(f'Timed out waiting for {label}: {diagnostic()}')


def run(protocol, interruption, logs, launch):
    import rosgraph
    import rospy
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
    from geometry_msgs.msg import Twist
    from move_base_msgs.msg import MoveBaseActionGoal
    from sensor_msgs.msg import NavSatFix
    from slicer_air_ground_msgs.msg import Command, GeoPoint
    from std_msgs.msg import Bool, String
    from tf2_msgs.msg import TFMessage
    from cup_gnss.enu import to_enu
    from test_odom_navigation import TestOnlyDifferentialPlant

    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
    from geographic_core import enu_to_geodetic

    rospy.init_node('slicer_c30d_driver', argv=[sys.argv[0]])
    origin = dict(latitude=31., longitude=118., altitude_ellipsoid_m=20.)
    yaw_map = .23
    c, s = math.cos(yaw_map), math.sin(yaw_map)

    def wgs84(local_x, local_y):
        return enu_to_geodetic((3. + c * local_x - s * local_y,
                               2. + s * local_x + c * local_y, 0.), origin)

    class Plant(TestOnlyDifferentialPlant):
        ROOM = (-5., 5., -3., 3.)

        def __init__(self):
            self.fix_pub = rospy.Publisher('/gnss/fix', NavSatFix, queue_size=10)
            self.quality_pub = rospy.Publisher('/gnss/fix_quality', DiagnosticArray, queue_size=10)
            self.ready_pub = rospy.Publisher('/slicer_c30d_driver/ready', Bool, queue_size=1)
            super().__init__('/cmd_vel')

        def _publish(self, state, scan_enabled, wall_x, obstacle_segments=()):
            # The antenna follows the configured nonzero horizontal lever.
            stamp = rospy.Time.now()
            super()._publish(state, scan_enabled, wall_x, obstacle_segments)
            x, y, yaw, _, _ = state
            lat, lon, height = wgs84(x + .065 * math.cos(yaw), y + .065 * math.sin(yaw))
            fix = NavSatFix(latitude=lat, longitude=lon, altitude=height)
            fix.header.stamp, fix.header.frame_id = stamp, 'gnss_link'
            fix.status.status = 2
            quality = DiagnosticArray(status=[DiagnosticStatus(name='cup_gnss/receiver', values=[
                KeyValue('gga_quality', '4'), KeyValue('fresh_rtk_usable', 'true'),
                KeyValue('fix_valid', 'true'), KeyValue('session', 'TEST_ONLY')])])
            quality.header.stamp = stamp
            self.fix_pub.publish(fix)
            self.quality_pub.publish(quality)
            self.ready_pub.publish(Bool(data=True))

    state_topic = '/air_ground/state' if protocol == 'v1' else '/air_ground/v2/state'
    command_topic = '/air_ground/command' if protocol == 'v1' else '/air_ground/v2/command'
    state, goals, mux_ids, transform_owners, raw_sources = {}, [], set(), {}, set()

    def on_state(message, key):
        record = json.loads(message.data)
        state[key] = record
        if key == 'mux':
            mux_ids.add(record['mux_instance_id'])

    def on_goal(message):
        goal = message.goal.target_pose
        goals.append((goal.header.frame_id, goal.pose.position.x, goal.pose.position.y,
                      message._connection_header['callerid']))

    def on_transform(message):
        for transform in message.transforms:
            edge = (transform.header.frame_id, transform.child_frame_id)
            transform_owners.setdefault(edge, set()).add(message._connection_header['callerid'])

    def on_velocity(message, key):
        if abs(message.linear.x) > .01:
            raw_sources.add((key, message._connection_header['callerid']))

    subscriptions = [
        rospy.Subscriber(topic, String, on_state, callback_args=key, queue_size=20)
        for key, topic in (('task', state_topic), ('geo', '/geographic/status'),
                           ('mux', '/cup_nav/manual_state'), ('runtime', '/geographic/navigation_runtime'))
    ]
    subscriptions.extend([rospy.Subscriber('/ugv/move_base/goal', MoveBaseActionGoal, on_goal, queue_size=10),
                          rospy.Subscriber('/tf', TFMessage, on_transform, queue_size=20)])
    subscriptions.extend(rospy.Subscriber(topic, Twist, on_velocity, callback_args=topic, queue_size=20)
                         for topic in ('/cmd_vel/nav_raw', '/cmd_vel/nav', '/cmd_vel'))
    publisher = rospy.Publisher(command_topic, Command if protocol == 'v1' else String, queue_size=10)
    seq = 0

    def send(operation, task='TEST_ONLY_first', point=None):
        nonlocal seq
        seq += 1
        if protocol == 'v1':
            msg = Command(protocol_version=1, server_id=state['task']['server_id'], sequence=seq,
                          request_id=f'TEST_ONLY_{seq}', task_id=task, operation=operation)
            if point:
                msg.final_goal = GeoPoint(latitude=point[0], longitude=point[1])
            matching = lambda reply: reply.get('sequence') == seq
        else:
            payload = dict(version=2, seq=seq, sent_at=rospy.Time.now().to_sec(),
                           task_id=task, op=operation)
            if point:
                payload['target'] = dict(lat=point[0], lon=point[1])
            msg = String(data=json.dumps(payload))
            matching = lambda reply: reply.get('seq') == seq
        publisher.publish(msg)
        reply = wait_until(lambda: (state['task']['last_command'] if matching(
            state.get('task', {}).get('last_command') or {}) else None), operation + ' receipt',
            diagnostic=lambda: state.get('task'))
        assert reply['accepted'], reply

    plant = Plant()

    def evidence():
        snapshot = dict(state)
        for k in ('geo', 'mux'):
            value = snapshot.get(k)
            if value:
                keys = ('state', 'reason', 'registered', 'alignment_progress', 'selected_source',
                        'manual_hold', 'hold_reason', 'last_intent_reason')
                snapshot[k] = {key: value[key] for key in keys if key in value}
        return dict(snapshot, goals=list(goals), pose=plant.pose(), model_error=plant.error,
                    velocity_sources=sorted(raw_sources), logs=str(logs))

    try:
        launch('geographic', 'roslaunch', 'ugv_bringup', 'slicer_geographic.launch',
               f'session_file:={logs / "session.json"}')
        launch('runtime', 'roslaunch', 'ugv_bringup', 'slicer_navigation_runtime.launch')
        if protocol == 'v1':
            launch('task', 'roslaunch', 'ugv_bringup', 'slicer_air_ground.launch')
        else:
            launch('task', 'roslaunch', 'ag_ugv_task', 'ugv.launch', 'monitor_only:=false', 'start_fkie:=false')
        wait_until(lambda: (state.get('runtime', {}).get('mode') == 'odom'
                   and state.get('geo', {}).get('paired_rtk_usable')
                   and state.get('mux', {}).get('manual_hold') is False
                   and plant.grid('global_costmap') is not None
                   and state.get('task', {}).get('phase') == 'IDLE'
                   and publisher.get_num_connections()), 'released odom startup and qualified RTK', 32., evidence)
        assert not state['geo']['registered'] and not goals

        set_op = 'SET_TASK' if protocol == 'v1' else 'SET_TARGET'
        initial, destination = wgs84(2.85, 0.), wgs84(2.35, 0.)
        send(set_op, point=initial)
        wait_until(lambda: state.get('runtime', {}).get('phase') == 'PREPARING',
                   'bounded odom preparation motion', 18., evidence)
        assert goals and all(row[0] == 'odom' for row in goals), evidence()
        assert state['task'].get('active', state['task'].get('active_point')) is None, evidence()
        task_id = 'TEST_ONLY_latest' if protocol == 'v1' else 'TEST_ONLY_first'
        send(set_op, task_id, destination)
        if interruption == 'pause':
            wait_until(lambda: plant.pose()[0] > .45, 'motion before pause', 10., evidence)
            generation = state['geo']['generation']
            send('PAUSE', task_id)
            wait_until(lambda: state.get('task', {}).get('phase') == 'PAUSED', 'verified pause', 12., evidence)
            paused_position = plant.pose()
            assert state['mux']['manual_hold'] and all(row[0] == 'odom' for row in goals), evidence()
            time.sleep(.5)
            assert math.dist(plant.pose(), paused_position) < .001, evidence()
            send('RESUME', task_id)
            wait_until(lambda: state.get('runtime', {}).get('phase') == 'PREPARING',
                       'resumed bounded odom motion', 12., evidence)
            assert state['geo']['generation'] == generation, evidence()
        if interruption == 'cancel':
            wait_until(lambda: state.get('runtime', {}).get('phase') == 'SWITCHING_NAVIGATION',
                       'mode switch before cancellation', 30., evidence)
            send('CANCEL' if protocol == 'v1' else 'STOP', task_id)
            stopped = 'CANCELED' if protocol == 'v1' else 'STOPPED'
            wait_until(lambda: state.get('task', {}).get('phase') == stopped,
                       'confirmed cancellation during mode switch', 16., evidence)
            time.sleep(.5)
            assert state['geo']['registered'] and all(row[0] == 'odom' for row in goals), evidence()
            assert abs(plant.velocity) < .001 and abs(plant.omega) < .001, evidence()
            print(f'PASS {protocol}: canceled switching task with no map dispatch; evidence in {logs}')
            return
        wait_until(lambda: any(row[0] == 'map' for row in goals), 'automatic map dispatch', 52., evidence)
        assert state['geo']['registered'], evidence()
        alignment = state['geo']['alignment']
        assert abs(alignment['yaw'] - yaw_map) < .01 and alignment['samples'] >= 6, evidence()
        assert state['task']['task_id'] == task_id, evidence()
        real = [row for row in goals if row[0] == 'map']
        expected = to_enu(*destination, state['geo']['origin'])
        assert len(real) == 1 and math.dist(real[0][1:3], expected[:2]) < .01, evidence()
        assert real[0][3] == ('/air_ground_task' if protocol == 'v1' else '/ag_ugv_task'), evidence()
        assert all(row[3] == '/slicer_navigation_runtime' for row in goals if row[0] == 'odom'), evidence()
        wait_until(lambda: state.get('task', {}).get('phase') == 'COMPLETED', 'verified geographic arrival',
                   35., evidence)

        send(set_op, 'TEST_ONLY_second', wgs84(2.85, 0.))
        wait_until(lambda: state.get('task', {}).get('task_id') == 'TEST_ONLY_second'
                   and state.get('task', {}).get('phase') == 'COMPLETED', 'reuse registered map navigation',
                   35., evidence)
        publishers = dict(rosgraph.Master(rospy.get_name()).getSystemState()[0])
        expected_velocity = {('/cmd_vel/nav_raw', '/ugv/move_base'),
                             ('/cmd_vel/nav', '/slicer_nav_arc_guard'), ('/cmd_vel', '/slicer_velocity_mux')}
        assert raw_sources == expected_velocity and publishers['/cmd_vel'] == ['/slicer_velocity_mux'], evidence()
        assert publishers['/ugv/move_base/status'] == ['/ugv/move_base'], evidence()
        assert transform_owners[('odom', 'base_link')] == {'/slicer_c30d_driver'}, transform_owners
        assert transform_owners[('map', 'odom')] == {'/global_localization/ekf'}, transform_owners
        assert len(mux_ids) == 2 and len([row for row in goals if row[0] == 'map']) == 2, evidence()
        odom_goal_count = 2 if interruption == 'pause' else 1
        assert plant.error is None and len([row for row in goals if row[0] == 'odom']) == odom_goal_count, evidence()
        print(f'PASS {protocol}: {odom_goal_count} odom target(s), two map targets, latest intent, registered reuse, '
              f'single TF/Action/velocity ownership; evidence in {logs}')
    finally:
        plant.close()
        for sub in subscriptions:
            sub.unregister()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('protocol', choices=('v1', 'v2'))
    parser.add_argument('--interrupt', choices=('pause', 'cancel'))
    args = parser.parse_args()
    with socket.socket() as free:
        free.bind(('127.0.0.1', 0))
        port = free.getsockname()[1]
    logs = Path(tempfile.mkdtemp(prefix=f'slicer-auto-registration-{args.protocol}-'))
    uri = f'http://127.0.0.1:{port}'
    os.environ.update(ROS_MASTER_URI=uri, ROS_IP='127.0.0.1', ROS_HOME=str(logs), ROS_LOG_DIR=str(logs))
    os.environ.pop('ROS_HOSTNAME', None)
    processes = []

    def launch(label, *argv):
        with (logs / (label + '.log')).open('w') as output:
            process = subprocess.Popen(argv, stdout=output, stderr=subprocess.STDOUT, start_new_session=True)
        processes.append(process)

    def master_alive():
        try:
            return ServerProxy(uri).getPid('/TEST_ONLY_registration')[2]
        except OSError:
            return False

    try:
        launch('core', 'roscore', '-p', str(port))
        wait_until(master_alive, 'private loopback ROS master')
        run(args.protocol, args.interrupt, logs, launch)
    finally:
        for process in reversed(processes):
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=12.)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=3.)


if __name__ == '__main__':
    main()
