#!/usr/bin/env python3
"""Real ROS task transport with fake navigation, only in an isolated container.

No velocity or hardware input is generated. Synthetic WGS84 coordinates here
are TEST_ONLY and must never be copied into a vehicle command.
"""
from collections import deque
import copy
import io
import json
import math
from pathlib import Path
import subprocess
import sys
import threading
import time
import unittest


class AirGroundROS(unittest.TestCase):
    def setUp(self):
        self.assertTrue(Path('/.dockerenv').exists(), 'requires no-device Docker container')
        self.assertEqual({path.name for path in Path('/sys/class/net').iterdir()}, {'lo'},
                         'requires Docker --network none')
        self.assertEqual(rospy.get_name(), '/ugv/move_base', 'fake Action must own its normal status identity')
        self.lock = threading.RLock()
        self.stop = threading.Event()
        self.pose = [0., 0., 0.]
        self.moving = False
        self.state = None
        self.manual_state = None
        self.history = deque(maxlen=1000)
        self.goals = []
        self.cancel_gate = threading.Event()
        self.cancel_gate.set()
        self.cancel_threads = []
        self.sequence = 0
        self.origin = dict(latitude=31., longitude=118., altitude_ellipsoid_m=20.)
        self.geo_pub = rospy.Publisher('/geographic/status', String, queue_size=1)
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
        self.stale_result_pub = rospy.Publisher('/ugv/move_base/result', MoveBaseActionResult, queue_size=1)
        self.command_pub = rospy.Publisher('/air_ground/command', Command, queue_size=1, latch=False)
        self.broadcaster = tf2_ros.TransformBroadcaster()
        self.state_sub = rospy.Subscriber('/air_ground/state', String, self.receive_state, queue_size=20)
        self.manual_sub = rospy.Subscriber('/cup_nav/manual_state', String, self.receive_manual, queue_size=20)
        self.server = actionlib.ActionServer('/ugv/move_base', MoveBaseAction,
                                             self.receive_goal, self.receive_cancel, auto_start=False)
        self.server.start()
        self.thread = threading.Thread(target=self.publish_sources, daemon=True)
        self.thread.start()
        self.until(lambda: self.state is not None and self.manual_state is not None,
                   'task and mux startup snapshots')
        self.assertEqual(self.state['phase'], 'IDLE')
        self.assertTrue(self.manual_state['manual_hold'])
        self.assertEqual(self.manual_state['hold_reason'], 'startup')
        self.until(lambda: self.command_pub.get_num_connections() > 0, 'formal command subscriber')
        # A real existing operator CLI releases only this TEST_ONLY mux. The fake
        # Action status owner deliberately has its required /ugv/move_base name.
        released = subprocess.run(['rosrun', 'ugv_slicer_control', 'control.py', 'release'],
                                  text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=8)
        self.assertEqual(released.returncode, 0, released.stdout + released.stderr)
        self.until(lambda: self.manual_state and not self.manual_state['manual_hold'], 'simulated operator release')

    def tearDown(self):
        self.cancel_gate.set()
        self.stop.set()
        self.thread.join(timeout=2)
        for thread in self.cancel_threads:
            thread.join(timeout=1)
        self.state_sub.unregister()
        self.manual_sub.unregister()
        for publisher in (self.geo_pub, self.odom_pub, self.stale_result_pub, self.command_pub):
            publisher.unregister()

    def until(self, predicate, description, timeout=8.):
        deadline = time.monotonic() + timeout
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            if predicate():
                return
            time.sleep(.02)
        self.fail('{}; state={}; goals={}'.format(description, self.state, len(self.goals)))

    def receive_state(self, message):
        with self.lock:
            self.state = json.loads(message.data)
            self.history.append(copy.deepcopy(self.state))

    def receive_manual(self, message):
        with self.lock:
            self.manual_state = json.loads(message.data)

    def receive_goal(self, handle):
        with self.lock:
            record = dict(handle=handle, goal=handle.get_goal(), canceled=False, terminal=False)
            self.goals.append(record)
            handle.set_accepted('TEST_ONLY navigation accepted')

    def receive_cancel(self, handle):
        with self.lock:
            matching = [row for row in self.goals if row['handle'].get_goal_id().id == handle.get_goal_id().id]
            if not matching or matching[0]['terminal']:
                return
            record = matching[0]
            record['canceled'] = True

        def acknowledge():
            if not self.cancel_gate.wait(5.):
                return
            with self.lock:
                if not record['terminal']:
                    handle.set_canceled(MoveBaseResult(), 'TEST_ONLY delayed cancel acknowledged')
                    record['terminal'] = True
        thread = threading.Thread(target=acknowledge, daemon=True)
        self.cancel_threads.append(thread)
        thread.start()

    def publish_sources(self):
        while not self.stop.is_set() and not rospy.is_shutdown():
            now = rospy.Time.now()
            with self.lock:
                x, y, yaw = self.pose
                moving = self.moving
            status = dict(state='ready', registered=True, generation=1, datum='WGS84',
                          origin=self.origin, status_stamp=now.to_sec(), reason='',
                          latest_observation=dict(stamp=now.to_sec(), base_xy=[x, y],
                                                  gga_quality=4, fresh_rtk_usable=True,
                                                  covariance_assumed=False))
            self.geo_pub.publish(String(data=json.dumps(status)))
            odom = Odometry()
            odom.header.stamp, odom.header.frame_id, odom.child_frame_id = now, 'odom', 'base_link'
            odom.pose.pose.position.x, odom.pose.pose.position.y = x, y
            odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = math.sin(yaw / 2.), math.cos(yaw / 2.)
            odom.twist.twist.linear.x = .1 if moving else 0.
            self.odom_pub.publish(odom)
            transform = TransformStamped()
            transform.header.stamp, transform.header.frame_id, transform.child_frame_id = now, 'map', 'base_link'
            transform.transform.translation.x, transform.transform.translation.y = x, y
            transform.transform.rotation = odom.pose.pose.orientation
            self.broadcaster.sendTransform(transform)
            self.stop.wait(.04)

    def point(self, east, north, yaw=None, altitude=True):
        latitude, longitude, height = enu_to_geodetic([east, north, 0.], self.origin)
        point = GeoPoint(latitude=latitude, longitude=longitude,
                         has_altitude=altitude, altitude_ellipsoid_m=height)
        if yaw is not None:
            point.has_yaw, point.yaw = True, yaw
        return point

    def command(self, operation, task_id, final=None, vias=(), accepted=True):
        self.sequence += 1
        message = Command(protocol_version=1, server_id=self.state['server_id'], sequence=self.sequence,
                          request_id='TEST_ONLY_request_{}'.format(self.sequence),
                          task_id=task_id, operation=operation, via_points=list(vias))
        if final is not None:
            message.final_goal = final
        self.command_pub.publish(message)
        def received():
            receipt = self.state.get('last_command') or {}
            transport = self.state.get('transport_error') or {}
            return (receipt.get('request_id') == message.request_id or
                    transport.get('request_id') == message.request_id)
        self.until(received, 'receipt for ' + operation)
        if (self.state.get('transport_error') or {}).get('request_id') == message.request_id:
            self.assertFalse(accepted, self.state)
        else:
            self.assertIs(self.state['last_command']['accepted'], accepted, self.state)
        return message

    def wait_goals(self, count):
        self.until(lambda: len(self.goals) >= count, 'navigation goal {}'.format(count))
        self.assertEqual(len(self.goals), count, 'more than one navigation execution submitted')
        return self.goals[count - 1]

    def assert_goal_xy(self, record, x, y):
        pose = record['goal'].target_pose
        self.assertEqual(pose.header.frame_id, 'map')
        self.assertAlmostEqual(pose.pose.position.x, x, delta=.001)
        self.assertAlmostEqual(pose.pose.position.y, y, delta=.001)

    def set_pose(self, x, y, yaw=0., moving=False):
        with self.lock:
            self.pose, self.moving = [x, y, yaw], moving

    def finish(self, record):
        with self.lock:
            record['handle'].set_succeeded(MoveBaseResult(), 'TEST_ONLY Action success')
            record['terminal'] = True

    def cancel_task(self, task_id):
        self.command('CANCEL', task_id)
        self.until(lambda: self.state['phase'] == 'CANCELED', 'confirmed task cancellation')

    def delayed_switch(self, command, old, count, x, y):
        self.cancel_gate.clear()
        command()
        self.until(lambda: old['canceled'], 'old Action cancellation requested')
        time.sleep(.25)
        self.assertEqual(len(self.goals), count, 'new goal sent before old cancel terminal ACK')
        self.cancel_gate.set()
        new = self.wait_goals(count + 1)
        self.assert_goal_xy(new, x, y)
        return new

    def test_formal_task_transport_and_navigation_lifecycle(self):
        # Final Action success alone is insufficient while odometry is moving.
        self.command('SET_TASK', 'single', self.point(1., .5, altitude=False))
        final = self.wait_goals(1)
        self.assert_goal_xy(final, 1., .5)
        self.set_pose(1., .5, moving=True)
        time.sleep(.12)
        self.finish(final)
        time.sleep(.35)
        self.assertNotEqual(self.state['phase'], 'COMPLETED')
        self.set_pose(1., .5)
        self.until(lambda: self.state['phase'] == 'COMPLETED', 'final quiet odom and qualified RTK arrival')
        result = self.state['last_task_result']
        self.assertEqual(result['status'], 'COMPLETED')
        self.assertTrue(result['result']['stop']['confirmed'])
        self.assertTrue(result['result']['geographic']['confirmed'])

        # Two position-only vias advance by radius while Action remains active;
        # the fixture faces the opposite direction and never reports via success.
        self.command('SET_TASK', 'vias', self.point(4., .5), [self.point(2., .5), self.point(3., .5)])
        via_a = self.wait_goals(2)
        self.assert_goal_xy(via_a, 2., .5)
        self.cancel_gate.clear()
        self.set_pose(1.82, .5, yaw=math.pi)
        self.until(lambda: via_a['canceled'], 'via radius cancellation without yaw alignment')
        time.sleep(.25)
        self.assertEqual(len(self.goals), 2, 'via progression must await cancellation result')
        self.cancel_gate.set()
        via_b = self.wait_goals(3)
        self.assert_goal_xy(via_b, 3., .5)
        self.set_pose(2.82, .5, yaw=math.pi)
        next_final = self.wait_goals(4)
        self.assert_goal_xy(next_final, 4., .5)
        self.cancel_task('vias')

        # Replacing all remaining vias cancels the current one; an empty batch
        # retains the original final. SET_TASK then replaces that final entirely.
        self.command('SET_TASK', 'replace-vias', self.point(8., .5), [self.point(5., .5), self.point(6., .5)])
        old = self.wait_goals(5)
        replacement = self.delayed_switch(
            lambda: self.command('REPLACE_VIA_POINTS', 'replace-vias', vias=[self.point(7., .5)]),
            old, 5, 7., .5)
        emptied = self.delayed_switch(
            lambda: self.command('REPLACE_VIA_POINTS', 'replace-vias', vias=[]),
            replacement, 6, 8., .5)
        replaced = self.delayed_switch(
            lambda: self.command('SET_TASK', 'new-final', self.point(9., .5)), emptied, 7, 9., .5)
        self.assertEqual(self.state['last_task_result']['status'], 'REPLACED')
        self.assertEqual(self.state['last_task_result']['task_id'], 'replace-vias')
        # Replay an old terminal result after the new Action became active.
        stale = MoveBaseActionResult()
        stale.header.stamp = rospy.Time.now()
        stale.status.goal_id = emptied['handle'].get_goal_id()
        stale.status.status = GoalStatus.SUCCEEDED
        self.stale_result_pub.publish(stale)
        time.sleep(.15)
        self.assertEqual(self.state['task_id'], 'new-final')
        self.assertNotEqual(self.state['phase'], 'COMPLETED')

        # Pause retains intent; a new final/route during pause cannot move until
        # an explicit RESUME, which releases only the task's own simulated hold.
        self.command('PAUSE', 'new-final')
        self.until(lambda: self.state['phase'] == 'PAUSED', 'pause cancellation and quiet stop')
        self.assertTrue(replaced['canceled'])
        self.command('SET_TASK', 'paused-update', self.point(12., .5), [self.point(10., .5)])
        self.command('REPLACE_VIA_POINTS', 'paused-update', vias=[self.point(11., .5)])
        time.sleep(.2)
        self.assertEqual(len(self.goals), 8)
        self.assertEqual(self.state['phase'], 'PAUSED')
        self.command('RESUME', 'paused-update')
        resumed = self.wait_goals(9)
        self.assert_goal_xy(resumed, 11., .5)

        # Duplicate retransmission, stale sequence and malformed coordinate
        # batches all preserve the active route and navigation execution.
        updated = self.command('REPLACE_VIA_POINTS', 'paused-update', vias=[self.point(11.5, .5)])
        self.wait_goals(10)
        revision = self.state['revision']
        self.command_pub.publish(updated)
        time.sleep(.15)
        self.assertEqual(self.state['revision'], revision)
        self.assertEqual(len(self.goals), 10)
        stale_command = copy.deepcopy(updated)
        stale_command.request_id = 'TEST_ONLY_stale_sequence'
        self.command_pub.publish(stale_command)
        self.until(lambda: self.state['last_command'].get('request_id') == stale_command.request_id,
                   'stale-sequence rejection')
        self.assertFalse(self.state['last_command']['accepted'])
        # After another command's receipt, an older identical retry must expose
        # its original accepted result again without repeating route execution.
        self.command_pub.publish(updated)
        self.until(lambda: self.state['last_command'].get('request_id') == updated.request_id,
                   'cached receipt replay after a newer receipt')
        self.assertTrue(self.state['last_command']['accepted'])
        self.assertEqual(self.state['revision'], revision)
        self.assertEqual(len(self.goals), 10)
        invalid = self.point(11.7, .5)
        invalid.latitude = float('nan')
        rejected = self.command('REPLACE_VIA_POINTS', 'paused-update',
                                vias=[self.point(11.6, .5), invalid], accepted=False)
        corrected = copy.deepcopy(rejected)
        corrected.via_points[1] = self.point(11.8, .5)
        self.command_pub.publish(corrected)
        self.until(lambda: self.state['last_command'].get('request_id') == corrected.request_id and
                   not self.state['last_command']['accepted'] and
                   'different content' in self.state['last_command']['reason'],
                   'invalid request identity cannot be reused with corrected coordinates')
        self.assertEqual(self.state['revision'], revision)
        self.assertEqual(len(self.goals), 10)

        # Execution does not depend on a state observer or aircraft ACK. Remove
        # our only state subscriber, advance one via, then read the latest latch.
        self.state_sub.unregister()
        self.set_pose(11.35, .5, yaw=math.pi)
        no_observer_final = self.wait_goals(11)
        self.assert_goal_xy(no_observer_final, 12., .5)
        self.state = None
        self.state_sub = rospy.Subscriber('/air_ground/state', String, self.receive_state, queue_size=20)
        self.until(lambda: self.state is not None and self.state['task_id'] == 'paused-update' and
                   (self.state.get('active') or {}).get('kind') == 'final',
                   'reconnected latest task snapshot')
        self.assertEqual(self.state['active']['kind'], 'final')
        self.cancel_task('paused-update')
        self.command('RESUME', 'paused-update', accepted=False)
        time.sleep(.2)
        self.assertEqual(len(self.goals), 11)
        self.assertEqual(self.state['phase'], 'CANCELED')

        # Use a separate process so rospy's per-topic shared publisher object
        # cannot reuse the existing nonlatched transport connection.
        self.sequence += 1
        latched = Command(protocol_version=1, server_id=self.state['server_id'],
                          sequence=self.sequence, request_id='TEST_ONLY_latched_command',
                          task_id='latched-would-move', operation='SET_TASK',
                          final_goal=self.point(20., .5))
        buffer = io.BytesIO()
        latched.serialize(buffer)
        publisher_code = '''
import sys, time
import rospy
from slicer_air_ground_msgs.msg import Command
rospy.init_node('TEST_ONLY_latched_command_sender', anonymous=True, disable_signals=True)
publisher = rospy.Publisher('/air_ground/command', Command, queue_size=1, latch=True)
deadline = time.monotonic() + 5.
while publisher.get_num_connections() == 0 and time.monotonic() < deadline:
    time.sleep(.02)
if publisher.get_num_connections() == 0:
    raise RuntimeError('formal task subscriber unavailable')
publisher.publish(Command().deserialize(bytes.fromhex(sys.argv[1])))
time.sleep(.5)
'''
        process = subprocess.Popen([sys.executable, '-c', publisher_code, buffer.getvalue().hex()],
                                   stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        try:
            self.until(lambda: (self.state.get('transport_error') or {}).get('request_id') == latched.request_id and
                       'latched' in self.state['transport_error']['reason'], 'latched formal command rejection')
            output, errors = process.communicate(timeout=8)
            self.assertEqual(process.returncode, 0, output + errors)
        finally:
            if process.poll() is None:
                process.kill()
                process.communicate(timeout=2)
        time.sleep(.2)
        self.assertEqual(len(self.goals), 11)
        self.assertEqual(self.state['phase'], 'CANCELED')
        self.assertEqual(self.state['task_id'], 'paused-update')


if __name__ == '__main__':
    # control.py verifies the Action status publisher's identity. Set it before
    # rospy/actionlib import, preserving rostest's result-file arguments.
    sys.argv[:] = [arg for arg in sys.argv if not arg.startswith(('__name:=', '__ns:='))]
    sys.argv += ['__name:=move_base', '__ns:=/ugv']
    import actionlib
    from actionlib_msgs.msg import GoalStatus
    from geometry_msgs.msg import TransformStamped
    from move_base_msgs.msg import MoveBaseAction, MoveBaseActionResult, MoveBaseResult
    from nav_msgs.msg import Odometry
    import rospy
    import rostest
    from slicer_air_ground_msgs.msg import Command, GeoPoint
    from std_msgs.msg import String
    import tf2_ros
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
    from geographic_core import enu_to_geodetic
    rospy.init_node('move_base', argv=sys.argv, anonymous=False, disable_signals=True)
    rostest.rosrun('ugv_navigation', 'air_ground_transport', AirGroundROS)
