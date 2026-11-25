#!/usr/bin/env python3
"""TEST_ONLY real Noetic costmap/GetPlan probe on a fresh isolated master.

Starts a new loopback Noetic master and an ideal lidar/odometry plant.
Never sends goals, releases control or publishes velocity. The closed-ring scan is an explicit synthetic unreachable fixture,
not a replay or a claim about field sensor geometry. Service wall time includes
transport/planning/possible mutex waits; this probe cannot separate those waits.
"""
import argparse
import json
import math
import os
from pathlib import Path
import socket
import statistics
import sys
import tempfile
import threading
import time
import xmlrpc.client

import rosgraph
import roslaunch
import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetPlan, GetPlanRequest
from rosgraph_msgs.msg import Log
from sensor_msgs.msg import LaserScan
from sensor_msgs import point_cloud2
from std_msgs.msg import Header

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'src/ugv_navigation/test'))
from test_odom_navigation import TestOnlyDifferentialPlant  # noqa: E402


def wait_for(predicate, description, timeout=15.):
    deadline = time.monotonic() + timeout
    while not rospy.is_shutdown() and time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(.025)
    raise AssertionError('Timed out: ' + description)


def cell(grid, x, y):
    if grid is None:
        return None
    column = math.floor((x - grid.info.origin.position.x) / grid.info.resolution)
    row = math.floor((y - grid.info.origin.position.y) / grid.info.resolution)
    if not (0 <= column < grid.info.width and 0 <= row < grid.info.height):
        return None
    return grid.data[row * grid.info.width + column]


def summary(samples):
    if not samples:
        return dict(count=0)
    ordered = sorted(samples)
    return dict(count=len(samples), min=min(samples), median=statistics.median(samples),
                p95=ordered[min(len(ordered) - 1, math.ceil(.95 * len(ordered)) - 1)],
                max=max(samples))


def planner_process():
    uri = rosgraph.Master(rospy.get_name()).lookupNode('/ugv/move_base')
    with xmlrpc.client.ServerProxy(uri) as node:
        status, _, pid = node.getPid(rospy.get_name())
    if status != 1:
        raise RuntimeError('Could not identify isolated move_base process')
    return Path('/proc') / str(pid)


def process_usage(proc):
    fields = (proc / 'stat').read_text().split(') ', 1)[1].split()
    seconds = (int(fields[11]) + int(fields[12])) / os.sysconf('SC_CLK_TCK')
    resident = int((proc / 'statm').read_text().split()[1]) * os.sysconf('SC_PAGE_SIZE')
    return seconds, resident


class Probe:
    def __init__(self):
        self.lock = threading.Lock()
        self.maps = {}
        self.events = []
        self.logs = []
        self.sub = rospy.Subscriber('/ugv/move_base/global_costmap/costmap', OccupancyGrid,
                                    self.on_map, queue_size=1, buff_size=32000000)
        self.log_sub = rospy.Subscriber('/rosout_agg', Log, self.on_log, queue_size=100)
        self.plant = TestOnlyDifferentialPlant(command_topic='/TEST_ONLY/unused_velocity')
        self.static = tf2_ros.StaticTransformBroadcaster()
        transforms = []
        for parent, child, x in [('map', 'odom', 0.), ('base_link', 'online_depth_optical', 0.),
                                 ('map', 'TEST_ONLY_ring_sensor', 2.)]:
            transform = TransformStamped()
            transform.header.stamp = rospy.Time.now()
            transform.header.frame_id, transform.child_frame_id = parent, child
            transform.transform.translation.x = x
            transform.transform.rotation.w = 1.
            transforms.append(transform)
        self.static.sendTransform(transforms)
        self.scan = rospy.Publisher('/scan', LaserScan, queue_size=5)
        self.hits = rospy.Publisher('/online_depth/obstacles',
                                    point_cloud2.PointCloud2, queue_size=5)
        self.scan_mode = None
        self.publish_depth = False
        self.stop = threading.Event()
        self.thread = threading.Thread(target=self.publish, daemon=True)
        self.thread.start()

    def on_map(self, message):
        with self.lock:
            self.maps['latest'] = message
            self.events.append(dict(wall=time.monotonic(), stamp=message.header.stamp.to_sec()))

    def on_log(self, message):
        if message.name == '/ugv/move_base' and message.level >= Log.WARN:
            with self.lock:
                self.logs.append(dict(wall=time.monotonic(), stamp=message.header.stamp.to_sec(),
                                      level=message.level, message=message.msg))

    def grid(self):
        with self.lock:
            return self.maps.get('latest')

    def reset_maps(self):
        with self.lock:
            self.maps.clear()

    def publish(self):
        while not self.stop.wait(.05) and not rospy.is_shutdown():
            if self.publish_depth:
                header = Header(stamp=rospy.Time.now(), frame_id='online_depth_optical')
                self.hits.publish(point_cloud2.create_cloud_xyz32(header, [(1.025, .625, .2)]))
            if self.scan_mode in ('ring', 'missing'):
                scan = LaserScan()
                scan.header.stamp = rospy.Time.now()
                ring = self.scan_mode == 'ring'
                scan.header.frame_id = 'TEST_ONLY_ring_sensor' if ring else 'base_link'
                scan.angle_min, scan.angle_increment = -math.pi, 2. * math.pi / 720
                scan.angle_max = scan.angle_min + 719 * scan.angle_increment
                scan.scan_time, scan.range_min, scan.range_max = .05, .05, 3.5
                scan.ranges = [.55 if ring else float('inf')] * 720
                self.scan.publish(scan)

    def phase(self, label, goals=(), repetitions=3, settle=2.):
        proc = planner_process()
        start = time.monotonic()
        cpu_before, _ = process_usage(proc)
        time.sleep(settle)
        calls = []
        service = rospy.ServiceProxy('/ugv/move_base/make_plan', GetPlan)
        for x, y in goals:
            for repeat in range(repetitions):
                request = GetPlanRequest()
                for pose, px, py in ((request.start, 0., 0.), (request.goal, x, y)):
                    pose.header.frame_id = 'map'
                    pose.header.stamp = rospy.Time.now()
                    pose.pose.position.x, pose.pose.position.y = px, py
                    pose.pose.orientation.w = 1.
                before = time.monotonic()
                response = service(request)
                after = time.monotonic()
                calls.append(dict(goal=[x, y], repeat=repeat, start_wall=before,
                                  end_wall=after, wall_s=after-before,
                                  poses=len(response.plan.poses)))
        time.sleep(2.)
        cpu_after, rss = process_usage(proc)
        end = time.monotonic()
        with self.lock:
            events = [entry for entry in self.events if start <= entry['wall'] <= end]
            warnings = [entry for entry in self.logs if start <= entry['wall'] <= end]
        gaps = [b['stamp'] - a['stamp'] for a, b in zip(events, events[1:])]
        wall_gaps = [b['wall'] - a['wall'] for a, b in zip(events, events[1:])]
        return dict(label=label, start_wall=start, end_wall=end, calls=calls,
                    get_plan_wall_s=summary([call['wall_s'] for call in calls]),
                    costmap_header_gap_s=summary(gaps), callback_wall_gap_s=summary(wall_gaps),
                    move_base_process_cpu_percent=100. * (cpu_after - cpu_before) / (end - start),
                    move_base_resident_bytes=rss, costmap_events=events, move_base_warnings=warnings)

    def close(self):
        self.stop.set()
        self.thread.join(timeout=2.)
        self.plant.close()
        self.sub.unregister()
        self.log_sub.unregister()
        self.scan.unregister()
        self.hits.unregister()


def navigation(uuid, extent=None, depth=False):
    # Roslaunch merges this final extent override before move_base starts. All
    # other parameters come from the real field entry, including lidar layers.
    launch = ROOT / 'src/ugv_bringup/launch/slicer_field_navigation.launch'
    override = '' if extent is None else ''.join(
        '<param name="/ugv/move_base/global_costmap/{}" value="{}" type="double"/>'.format(key, value)
        for key, value in (('width', extent), ('height', extent),
                           ('origin_x', -extent/2.), ('origin_y', -extent/2.)))
    text = '''<launch>
      <include file="{launch}">
        <arg name="enable_depth" value="{depth}"/>
        <arg name="start_camera" value="false"/>
      </include>
      {override}
    </launch>'''.format(launch=launch, depth=str(depth).lower(), override=override)
    with tempfile.NamedTemporaryFile(mode='w', suffix='.launch') as file:
        file.write(text)
        file.flush()
        parent = roslaunch.parent.ROSLaunchParent(uuid, [file.name])
        parent.start()
    rospy.wait_for_service('/ugv/move_base/make_plan', timeout=20.)
    return parent


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    with socket.socket() as listener:
        listener.bind(('127.0.0.1', 0))
        port = listener.getsockname()[1]
    log_dir = args.output.resolve().parent / 'ros-outdoor-probe'
    log_dir.mkdir(parents=True, exist_ok=True)
    os.environ.update(ROS_MASTER_URI=f'http://127.0.0.1:{port}', ROS_IP='127.0.0.1',
                      ROS_HOME=str(log_dir), ROS_LOG_DIR=str(log_dir))
    os.environ.pop('ROS_HOSTNAME', None)
    result = dict(TEST_ONLY=True, platform=__import__('platform').machine()+' Noetic', network='loopback',
                  devices='none', sends_goals=False, sends_velocity=False,
                  measurements='GetPlan service wall time, global costmap gaps and move_base process CPU/RSS',
                  limitation='No real sensor processing, motion, or lock-wait attribution',
                  status='starting', profiles=[])

    def save():
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')

    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    core = roslaunch.parent.ROSLaunchParent(uuid, [], is_core=True, port=port)
    parent = probe = None
    save()
    try:
        core.start()
        rospy.init_node('TEST_ONLY_outdoor_costmap_probe', anonymous=True)
        probe = Probe()
        # Demonstrate depth contribution, then stop/rebuild the actual field
        # navigation entry. A separate sentinel proves cleanup is node-scoped.
        rospy.set_param('/TEST_ONLY_survives_navigation_restart', 'unchanged')
        probe.publish_depth = True
        parent = navigation(uuid, depth=True)
        wait_for(lambda: cell(probe.grid(), 1.025, .625) == 100, 'synthetic depth cell marked')
        result['depth_on'] = dict(cell=cell(probe.grid(), 1.025, .625),
                                 depth_params_present=rospy.has_param('/ugv/move_base/global_costmap/depth_layer'))
        probe.publish_depth = False
        parent.shutdown()
        parent = None
        probe.reset_maps()
        probe.plant.set_scan(False)
        probe.scan_mode = 'missing'
        parent = navigation(uuid)
        wait_for(lambda: cell(probe.grid(), 1.025, .625) == -1, 'empty projection leaves cell unknown')
        time.sleep(1.)
        assert cell(probe.grid(), 1.025, .625) == -1
        probe.scan_mode = None
        probe.plant.set_scan(True)
        wait_for(lambda: cell(probe.grid(), 1.025, .625) == 0, 'fresh depth-off grid free at prior depth hit')
        result['depth_off_restart'] = dict(cell=cell(probe.grid(), 1.025, .625),
            depth_params_present=rospy.has_param('/ugv/move_base/global_costmap/depth_layer'),
            sentinel=rospy.get_param('/TEST_ONLY_survives_navigation_restart'))
        assert not result['depth_off_restart']['depth_params_present']
        assert result['depth_off_restart']['sentinel'] == 'unchanged'
        # Check actual lidar marking and clearing on the rebuilt depth-off grid.
        probe.plant.set_segments([((1.025, -.3), (1.025, .3))])
        wait_for(lambda: cell(probe.grid(), 1.025, .025) == 100, 'radar marking after depth-off restart')
        marked = cell(probe.grid(), 1.025, .025)
        probe.plant.set_scan(False)
        probe.scan_mode = 'missing'
        time.sleep(1.)
        missing = cell(probe.grid(), 1.025, .025)
        assert missing == 100
        probe.scan_mode = None
        probe.plant.set_segments([])
        probe.plant.set_scan(True)
        wait_for(lambda: cell(probe.grid(), 1.025, .025) == 0, 'radar clearing after segment removal')
        result['lidar_after_restart'] = dict(marked=marked, after_missing=missing, cleared=cell(probe.grid(), 1.025, .025))
        parent.shutdown()
        parent = None

        for extent in (40., 70., 90.):
            probe.plant.set_scan(True)
            probe.scan_mode = None
            probe.reset_maps()
            parent = navigation(uuid, extent=extent)
            wait_for(lambda: cell(probe.grid(), .8, 0.) == 0, 'free reachable target')
            grid = probe.grid()
            profile = dict(extent_m=extent,
                           full_grid_payload_bytes=grid.info.width * grid.info.height,
                           cells=[grid.info.width, grid.info.height],
                           frame=grid.header.frame_id,
                           global_parameters=rospy.get_param('/ugv/move_base/global_costmap'),
                           phases=[])
            profile['phases'].append(probe.phase('reachable', [(0.8, 0.)]))
            assert all(call['poses'] > 2 for call in profile['phases'][-1]['calls'])
            # A synthetic scanner inside a closed ring observes its complete
            # boundary. Keep it live while asking the real planner from outside.
            probe.plant.set_scan(False)
            probe.scan_mode = 'ring'
            wait_for(lambda: cell(probe.grid(), 1.475, .025) >= 99, 'closed-ring near wall')
            wait_for(lambda: cell(probe.grid(), 2.525, .025) >= 99, 'closed-ring far wall')
            profile['phases'].append(probe.phase('surrounded_goal', [(2., 0.)]))
            assert all(call['poses'] == 0 for call in profile['phases'][-1]['calls'])
            profile['phases'].append(probe.phase('out_of_bounds', [(extent/2. + 1., 0.)]))
            assert all(call['poses'] == 0 for call in profile['phases'][-1]['calls'])
            result['profiles'].append(profile)
            save()
            parent.shutdown()
            parent = None
        result['status'] = 'collected'
        save()
    except Exception as error:
        result.update(status='failed', error=repr(error))
        if probe is not None and probe.grid() is not None:
            grid = probe.grid()
            result['failure_grid'] = dict(frame=grid.header.frame_id,
                origin=[grid.info.origin.position.x, grid.info.origin.position.y],
                cells=[grid.info.width, grid.info.height],
                depth_neighborhood=[[x, y, cell(grid, x, y)]
                    for x in (.95, 1., 1.05) for y in (.55, .6, .65)])
        save()
        raise
    finally:
        if parent is not None:
            parent.shutdown()
        if probe is not None:
            probe.close()
        core.shutdown()


if __name__ == '__main__':
    main()
