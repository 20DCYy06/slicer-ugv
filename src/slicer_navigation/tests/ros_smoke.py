#!/usr/bin/env python3
"""Real AMCL/GlobalPlanner/TEB on a synthetic obstacle map; no chassis or sensors."""
import math
import json
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import xmlrpc.client
import yaml

PKG = Path(__file__).resolve().parents[1]
children = []


def until(predicate, timeout=15, label='condition'):
    deadline = time.monotonic()+timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(.05)
    raise AssertionError('Timeout waiting for ' + label)


def main():
    with tempfile.TemporaryDirectory(prefix='slicer-nav-') as directory:
        folder = Path(directory)
        handles = []
        def launch(command, name, env=None):
            handle = (folder/(name+'.log')).open('w')
            handles.append(handle)
            child = subprocess.Popen(command, stdout=handle, stderr=subprocess.STDOUT, env=env)
            children.append(child)
            return child
        launch(['roscore'], 'master')
        proxy = xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI'])
        def master_ready():
            try:
                return proxy.getPid('/navigation_smoke')[0] == 1
            except OSError:
                return False
        until(master_ready, label='ROS master')
        import actionlib
        import rospy
        import tf
        from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
        from nav_msgs.msg import OccupancyGrid, Odometry, Path as RosPath
        from nav_msgs.srv import GetPlan, GetPlanRequest
        from std_msgs.msg import String
        from sensor_msgs.msg import LaserScan
        from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
        from tf.transformations import quaternion_from_euler
        from visualization_msgs.msg import MarkerArray
        rospy.init_node('navigation_smoke', disable_signals=True)
        yaw = 2.1  # More than 120 degrees away from the goal bearing.
        front_obstacle = [None]
        q = quaternion_from_euler(0, 0, yaw)
        broadcaster = tf.TransformBroadcaster()
        map_pub = rospy.Publisher('/map', OccupancyGrid, queue_size=1, latch=True)
        scan_pub = rospy.Publisher('/scan', LaserScan, queue_size=1)
        odom_pub = rospy.Publisher('/odom', Odometry, queue_size=1)
        grid = OccupancyGrid()
        grid.header.frame_id = 'map'
        grid.info.resolution, grid.info.width, grid.info.height = .05, 120, 120
        grid.info.origin.position.x = grid.info.origin.position.y = -3.
        grid.info.origin.orientation.w = 1.
        # A real lethal obstacle separates robot and goal; global plan must detour.
        grid.data = [100 if x in (0,119) or y in (0,119) or (71 <= x <= 75 and 57 <= y <= 62)
                     else 0 for y in range(120) for x in range(120)]
        map_pub.publish(grid)

        def sensors(unused):
            now = rospy.Time.now()
            broadcaster.sendTransform((0, 0, 0), q, now, 'base_link', 'odom')
            broadcaster.sendTransform((.07, 0, .18), (0,0,0,1), now, 'laser_link', 'base_link')
            scan = LaserScan()
            scan.header.stamp, scan.header.frame_id = now, 'laser_link'
            scan.angle_min, scan.angle_max, scan.angle_increment = -math.pi, math.pi, math.pi/90
            scan.range_min, scan.range_max, scan.scan_time = .05, 6., .05
            ranges = []
            sx, sy = .07*math.cos(yaw), .07*math.sin(yaw)
            for index in range(181):
                angle = yaw+scan.angle_min+index*scan.angle_increment
                distance = .05
                while distance < 6.:
                    x, y = sx+distance*math.cos(angle), sy+distance*math.sin(angle)
                    ix, iy = int((x+3.)/.05), int((y+3.)/.05)
                    if not (0 <= ix < 120 and 0 <= iy < 120) or grid.data[iy*120+ix] == 100:
                        break
                    distance += .05
                ranges.append(distance)
            scan.ranges = ranges
            if front_obstacle[0] is not None:
                scan.ranges[90] = front_obstacle[0]
            scan_pub.publish(scan)
            odom = Odometry()
            odom.header.stamp, odom.header.frame_id, odom.child_frame_id = now, 'odom', 'base_link'
            (odom.pose.pose.orientation.x, odom.pose.pose.orientation.y,
             odom.pose.pose.orientation.z, odom.pose.pose.orientation.w) = q
            odom_pub.publish(odom)
        timer = rospy.Timer(rospy.Duration(.1), sensors)
        saver = launch(['bash', str(PKG/'scripts/save_map.sh'), 'test_map'], 'save',
                       dict(os.environ, SLICER_MAP_DIR=str(folder)))
        until(lambda: saver.poll() is not None, label='map save')
        assert saver.returncode == 0
        saver = launch(['bash', str(PKG/'scripts/save_map.sh'), 'test_map'], 'replace',
                       dict(os.environ, SLICER_MAP_DIR=str(folder)))
        until(lambda: saver.poll() is not None, label='map replace')
        assert saver.returncode == 0 and list((folder/'.backups').iterdir())
        map_pub.unregister()
        nav = launch(['roslaunch', 'slicer_navigation', 'nav.launch', 'root:=/slicer',
                      'map_file:='+str(folder/'test_map.yaml'), 'start_sensors:=false',
                      'start_control:=false', 'gui:=false', 'set_initial_pose:=true',
                      'initial_yaw:='+str(yaw)], 'nav')
        rospy.wait_for_service('/move_base/TebLocalPlannerROS/set_parameters', timeout=20)
        listener = tf.TransformListener()
        listener.waitForTransform('map', 'base_link', rospy.Time(0), rospy.Duration(10))
        markers = rospy.wait_for_message('/slicer/navigation_markers', MarkerArray, timeout=5)
        assert len(markers.markers) == 4
        assert rospy.get_param('/move_base/GlobalPlanner/use_grid_path') is True
        assert rospy.get_param('/move_base/base_global_planner') == 'slicer_navigation/BSplineGlobalPlanner'
        assert rospy.get_param('/move_base/GlobalPlannerRaw/use_dijkstra') is True
        assert rospy.get_param('/move_base/GlobalPlanner/old_navfn_behavior') is True
        assert rospy.get_param('/move_base/GlobalPlanner/allow_unknown') is True
        assert rospy.get_param('/move_base/TebLocalPlannerROS/min_turning_radius') == 0.
        assert rospy.get_param('/move_base/TebLocalPlannerROS/feasibility_check_no_poses') == 3
        raw, smooth, local, global_plan = [], [], [], []
        raw_paths, spline_status = [], []
        subscribers = [
            rospy.Subscriber('/cmd_vel/nav_raw', Twist, lambda m: raw.append((m.linear.x, m.angular.z))),
            rospy.Subscriber('/cmd_vel/nav', Twist, lambda m: smooth.append((m.linear.x, m.angular.z))),
            rospy.Subscriber('/move_base/TebLocalPlannerROS/local_plan', RosPath, lambda m: local.append(m)),
            rospy.Subscriber('/move_base/GlobalPlanner/plan', RosPath, lambda m: global_plan.append(m)),
            rospy.Subscriber('/move_base/GlobalPlanner/plan_raw', RosPath, lambda m: raw_paths.append(m)),
            rospy.Subscriber('/move_base/GlobalPlanner/smoothing_status', String, lambda m: spline_status.append(m.data))]
        client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        assert client.wait_for_server(rospy.Duration(10))
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id, goal.target_pose.header.stamp = 'map', rospy.Time.now()
        goal.target_pose.pose.position.x, goal.target_pose.pose.position.y = 1.3, .05
        goal.target_pose.pose.orientation.w = 1.
        client.send_goal(goal)
        try:
            until(lambda: (any(m.poses for m in local) and any(abs(v)+abs(w) > .01 for v,w in raw)
                           and any(abs(v)+abs(w) > .01 for v,w in smooth)),
                  timeout=20, label='TEB path and nonzero smoothed velocity with large initial yaw')
            assert global_plan and nav.poll() is None
            path = global_plan[-1]
            assert all(grid.data[int((p.pose.position.y+3.)/.05)*120+
                                      int((p.pose.position.x+3.)/.05)] < 100 for p in path.poses)
            assert max(abs(p.pose.position.y) for p in path.poses) > .2, 'Global path did not detour'
            max_w = rospy.get_param('/move_base/TebLocalPlannerROS/max_vel_theta')
            max_v = rospy.get_param('/move_base/TebLocalPlannerROS/max_vel_x')
            assert all(-1e-9 <= v <= max_v+1e-6 and abs(w) <= max_w+1e-6 for v,w in smooth)
            print('AMCL_RVIZ_GLOBAL_DETOUR_TEB_LARGE_YAW_NONZERO_PASS', flush=True)
        except Exception:
            print((folder/'nav.log').read_text(errors='replace')[-18000:], flush=True)
            raise
        finally:
            client.cancel_goal()
            client.wait_for_result(rospy.Duration(2))
        until(lambda: smooth and smooth[-1] == (0., 0.), timeout=3, label='cancelled goal zero speed')
        # Call the actual move_base plugin interface on an open-space goal. This
        # verifies the returned plan itself is smoothed, rather than a cosmetic relay.
        request=GetPlanRequest()
        request.start.header.frame_id=request.goal.header.frame_id='map'
        request.start.header.stamp=request.goal.header.stamp=rospy.Time.now()
        request.start.pose.orientation.w=request.goal.pose.orientation.w=1.
        request.goal.pose.position.x=-1.0
        request.goal.pose.position.y=.5
        before_status=len(spline_status)
        response=rospy.ServiceProxy('/move_base/make_plan',GetPlan)(request)
        until(lambda: len(spline_status)>before_status and spline_status[-1].startswith('smoothed '),label='spline result status')
        assert raw_paths and response.plan.poses
        poses=response.plan.poses
        assert math.hypot(poses[0].pose.position.x,poses[0].pose.position.y)<1e-8
        assert math.hypot(poses[-1].pose.position.x+1.,poses[-1].pose.position.y-.5)<1e-6
        assert abs(poses[-1].pose.orientation.w-1.)<1e-6
        lengths=[math.hypot(b.pose.position.x-a.pose.position.x,b.pose.position.y-a.pose.position.y)
                 for a,b in zip(poses,poses[1:])]
        assert max(lengths)<.051, lengths
        evidence=os.environ.get('SLICER_BSPLINE_EVIDENCE')
        if evidence:
            def xy(path):
                return [[p.pose.position.x,p.pose.position.y] for p in path.poses]
            target=Path(evidence);target.parent.mkdir(parents=True,exist_ok=True)
            target.write_text(json.dumps({'synthetic_map':True,'raw':xy(raw_paths[-1]),
                                         'smoothed':xy(response.plan),'status':spline_status[-1]},indent=2))
        print('BSPLINE_PLUGIN_MAKE_PLAN_AND_TEB_INTERFACE_PASS '+spline_status[-1],flush=True)
        # Explicit pure-spin input must survive without TF/costmap vetoes.
        spin_pub = rospy.Publisher('/cmd_vel/patrol_spin_raw', Twist, queue_size=1)
        until(lambda: spin_pub.get_num_connections() > 0, label='spin relay subscriber')
        smooth.clear()
        spin = Twist()
        spin.angular.z = .3
        limits = yaml.safe_load((PKG.parents[2]/'config/slicer_config.yaml').read_text())['navigation']
        body_control = yaml.safe_load((PKG.parents[2]/'catkin_ws/src/slicer_wasd_control/config/teleop_params.yaml').read_text()).get('control_mode') == 'body_speed'
        turn_start = spin.angular.z if body_control else limits['turn_start_angular']
        # 现场起转角速已提高，等待时长应覆盖现有加速度斜坡，不能固定1.25秒。
        spin_samples = max(25, int((turn_start/limits['accel_angular']+.4)/.05)+1)
        for unused in range(spin_samples):
            spin_pub.publish(spin)
            time.sleep(.05)
        assert any(v == 0. and w > .1 for v,w in smooth), smooth
        assert any(v == 0. and w >= turn_start-1e-3 for v,w in smooth), smooth
        time.sleep(.7)
        assert smooth[-1] == (0., 0.)
        # Reproduce the field's negative micro-vx/weak-turn request.
        smooth.clear()
        spin.linear.x, spin.angular.z = -.004, -.384
        for unused in range(spin_samples):
            spin_pub.publish(spin)
            time.sleep(.05)
        assert smooth and all(v == 0. for v,w in smooth), smooth
        reverse_turn = .384 if body_control else turn_start
        assert any(w <= -reverse_turn+1e-3 for v,w in smooth), smooth
        spin_pub.publish(Twist())
        time.sleep(.6)
        assert smooth[-1] == (0., 0.)
        print('NO_REVERSE_TURN_COMPENSATION_PASS', flush=True)
        print('PURE_SPIN_SMOOTHING_WATCHDOG_PASS', flush=True)
        if body_control:
            # 同时存在前进与转向时应保留曲线；小角速不被旧开环起转门槛抬高。
            smooth.clear()
            spin.linear.x, spin.angular.z = .02, .12
            for unused in range(25):
                spin_pub.publish(spin)
                time.sleep(.05)
            assert any(abs(v-.02)<1e-4 and abs(w-.12)<1e-4 for v,w in smooth), smooth
            assert all(abs(w)<=.120001 for v,w in smooth), smooth
            spin_pub.publish(Twist());time.sleep(.6)
            print('BODY_CONTINUOUS_CURVE_NO_OPEN_LOOP_BOOST_PASS', flush=True)
        # 仍在隔离容器内：给既有出口输入前进速度和15cm雷达回波，验证零速输出。
        smooth.clear()
        front_obstacle[0] = .5
        spin.linear.x, spin.angular.z = .1, 0.
        for unused in range(15):
            spin_pub.publish(spin)
            time.sleep(.05)
        assert any(v > .02 for v,w in smooth), smooth
        front_obstacle[0] = .15
        for unused in range(20):
            spin_pub.publish(spin)
            time.sleep(.05)
        assert all(v == 0. and w == 0. for v,w in smooth[-5:]), smooth[-10:]
        spin_pub.publish(Twist())
        front_obstacle[0] = None
        print('FRONT_LASER_15CM_STOP_PASS', flush=True)
        timer.shutdown()
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
