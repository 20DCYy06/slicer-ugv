#!/usr/bin/env python3
"""Isolated XY latch -> cancel -> pure yaw; no action success or XY re-centering."""
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import xmlrpc.client
import actionlib
import rospy
import rosparam
import tf
import yaml
from move_base_msgs.msg import MoveBaseAction, MoveBaseResult
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Path as RosPath
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from std_srvs.srv import Empty, EmptyResponse
from tf.transformations import quaternion_from_euler, euler_from_quaternion

PACKAGE = Path(__file__).resolve().parents[1]
children = []


def wait_for(predicate, seconds, label):
    deadline = time.monotonic()+seconds
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(.05)
    raise AssertionError('Timed out waiting for ' + label)


def main():
    with tempfile.TemporaryDirectory(prefix='slicer-patrol-') as directory:
        folder = Path(directory)
        with (folder/'master.log').open('w') as master_log:
            children.append(subprocess.Popen(['roscore'], stdout=master_log, stderr=subprocess.STDOUT))
            proxy = xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI'])
            def master_ready():
                try:
                    return proxy.getPid('/patrol_smoke')[0] == 1
                except OSError:
                    return False
            wait_for(master_ready, 10, 'ROS master')
            rospy.init_node('patrol_smoke', disable_signals=True)
            pose, goals, clears = [0., 0., 0.], [], []
            spin_commands, shifted, cancellations = [], set(), []
            mission_commands = []
            phase, departure_unblocked = [''], [False]
            spin_target, last_tick = [0., -math.inf], [time.monotonic()]
            broadcaster = tf.TransformBroadcaster()
            ready = rospy.Publisher('/smoke/ready', Bool, queue_size=1, latch=True)
            scan = rospy.Publisher('/scan', LaserScan, queue_size=1)
            costmap = rospy.Publisher('/move_base/local_costmap/costmap', OccupancyGrid, queue_size=1)
            local_plan = rospy.Publisher('/move_base/TebLocalPlannerROS/local_plan', RosPath, queue_size=1)

            def spin(msg):
                spin_commands.append((msg.linear.x, msg.angular.z))
                spin_target[:] = [msg.angular.z, time.monotonic()]
            rospy.Subscriber('/smoke/spin', Twist, spin, queue_size=1)
            def state(msg):
                phase[0] = msg.data
            rospy.Subscriber('/smoke_patrol/state', String, state, queue_size=1)
            def mission(msg):
                mission_commands.append((time.monotonic(), msg.linear.x, msg.angular.z))
                if msg.linear.x < 0:
                    assert not server.is_active(), 'Backup before cancellation acknowledgement'
                    if len(goals) == 2:
                        departure_unblocked[0] = True
            rospy.Subscriber('/smoke/stop', Twist, mission, queue_size=1)

            def tick(unused):
                wall = time.monotonic()
                dt = min(.1, wall-last_tick[0])
                last_tick[0] = wall
                if wall-spin_target[1] < .2 and spin_target[0]:
                    assert not server.is_active(), 'Spin started before goal cancellation acknowledgement'
                    # Near-obstacle regression: departure cannot rotate until
                    # one backup opens room. Final yaw on the previous point
                    # must still work normally.
                    blocked = phase[0] == 'ALIGN_DEPARTURE second' and not departure_unblocked[0]
                    if not blocked:
                        pose[2] += spin_target[0]*dt
                    if len(goals) not in shifted:
                        # Reproduce an XY localization shift after arriving. Yaw must
                        # finish without creating a second position action goal.
                        pose[0] += .25
                        shifted.add(len(goals))
                now = rospy.Time.now()
                broadcaster.sendTransform((pose[0], pose[1], 0), quaternion_from_euler(0, 0, pose[2]),
                                          now, 'test_base', 'map')
                ready.publish(True)
                message = LaserScan()
                message.header.stamp, message.header.frame_id = now, 'test_base'
                scan.publish(message)
                grid = OccupancyGrid()
                grid.header.stamp, grid.header.frame_id = now, 'map'
                costmap.publish(grid)
                if len(goals) == 1:
                    # Field regression: a fresh nonempty TEB band exists, but
                    # the chassis does not translate or rotate. Still recover.
                    band = RosPath()
                    band.header.stamp, band.header.frame_id = now, 'map'
                    band.poses = [PoseStamped(), PoseStamped()]
                    local_plan.publish(band)
            timer = rospy.Timer(rospy.Duration(.05), tick)

            def execute(goal):
                xyz, q = goal.target_pose.pose.position, goal.target_pose.pose.orientation
                yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
                goals.append((xyz.x, xyz.y, yaw))
                if len(goals) == 1:
                    while not rospy.is_shutdown() and not server.is_preempt_requested():
                        time.sleep(.02)
                    time.sleep(3.2)  # Reproduce a controller busy longer than the old 2s deadline.
                    server.set_preempted()
                    return
                pose[:] = [xyz.x, xyz.y, yaw+.8]
                while not rospy.is_shutdown() and not server.is_preempt_requested():
                    time.sleep(.02)
                cancellations.append(len(goals))
                server.set_preempted()  # Never SUCCEEDED: XY entry alone must latch.
            server = actionlib.SimpleActionServer('/mock_move_base', MoveBaseAction, execute_cb=execute, auto_start=False)
            server.start()
            rospy.set_param('/move_base/TebLocalPlannerROS/max_vel_theta', .9)

            def clear(unused):
                assert not server.is_active(), 'Clear called while old goal still active'
                assert mission_commands and mission_commands[-1][1:] == (0., 0.), 'Clear before backup stop'
                clears.append(time.monotonic())
                time.sleep(.2)  # Blocking service must not block the patrol event loop.
                return EmptyResponse()
            service = rospy.Service('/move_base/clear_costmaps', Empty, clear)
            points = folder/'waypoints.yaml'
            points.write_text(yaml.safe_dump({'frame_id': 'map', 'waypoints': [
                {'name': 'first', 'x': 1., 'y': 0., 'yaw': .5},
                {'name': 'second', 'x': 0., 'y': 1., 'yaw': -.7}]}))
            params = yaml.safe_load((PACKAGE/'config/patrol_params.yaml').read_text())
            params.update(file=str(points), action_name='/mock_move_base', base_frame='test_base',
                          control_ready_topic='/smoke/ready', wait_for_localization=False,
                          goal_timeout=15., wait_s=0., auto_exit=True, stop_topic='/smoke/stop',
                          spin_topic='/smoke/spin')
            rosparam.upload_params('/smoke_patrol', params)
            with (folder/'patrol.log').open('w') as patrol_log:
                child = subprocess.Popen(['rosrun', 'slicer_navigation', 'multi_goal_patrol.py',
                                          '__name:=smoke_patrol'], stdout=patrol_log, stderr=subprocess.STDOUT)
                children.append(child)
                wait_for(lambda: child.poll() is not None, 35, 'backup recovery and two points')
            output = (folder/'patrol.log').read_text(errors='replace')
            assert child.returncode == 0, output
            assert len(clears) == 2 and len(goals) == 3, (clears, goals, output)
            assert goals[0] == goals[1], goals
            assert abs(goals[1][2]-.5) < .001 and abs(goals[2][2]+.7) < .001, goals
            assert 'RECOVERING first' in output and 'RECOVERED first' in output, output
            assert 'RECOVERY_TRIGGER no_local=False stalled=True' in output, output
            assert 'CANCEL_PENDING' in output and 'DIAG' in output, output
            assert 'REACHED first' in output and 'REACHED second' in output and 'COMPLETED' in output, output
            assert 'POSITION_REACHED first' in output and 'ALIGN_YAW first' in output, output
            assert 'POSITION_REACHED second' in output and 'ALIGN_YAW second' in output, output
            assert 'ALIGN_DEPARTURE first' in output and 'ALIGN_DEPARTURE second' in output, output
            assert 'Departure heading aligned' in output, output
            assert 'DEPARTURE_TURN_BLOCKED' in output and departure_unblocked[0], output
            backup = [item for item in mission_commands if item[1] < 0]
            assert backup and all(item[1:] == (-.20, 0.) for item in backup), backup
            batches = []
            for item in backup:
                if not batches or item[0]-batches[-1][-1][0] > .2:
                    batches.append([])
                batches[-1].append(item)
            assert len(batches) == 2, batches
            assert all(2.0 <= batch[-1][0]-batch[0][0] <= 2.5 for batch in batches), batches
            assert all(clear-batch[-1][0] >= .45 for clear,batch in zip(clears,batches))
            assert cancellations == [2,3] and shifted == {2,3}, (cancellations, shifted)
            assert spin_commands and all(v == 0. for v,w in spin_commands), spin_commands
            assert any(w != 0. for v,w in spin_commands), spin_commands
            assert abs(math.atan2(math.sin(pose[2]+.7), math.cos(pose[2]+.7))) < .3, pose
            print('ROS_PATROL_XY_LATCH_CANCEL_PURE_YAW_BACKUP_DEPARTURE_PASS', flush=True)
            timer.shutdown()
            service.shutdown()
            rospy.signal_shutdown('test complete')


try:
    main()
finally:
    for child in reversed(children):
        if child.poll() is None:
            child.send_signal(signal.SIGINT)
            try:
                child.wait(timeout=5)
            except subprocess.TimeoutExpired:
                child.kill()
                child.wait()
