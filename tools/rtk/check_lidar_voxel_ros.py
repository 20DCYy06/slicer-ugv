#!/usr/bin/env python3
"""Exercise the RTK voxel profile with synthetic clouds on a private master."""
import argparse
from contextlib import ExitStack
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import time




def wait_for(predicate, description, timeout=10.):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(.05)
    raise AssertionError(description)


def stop(process):
    if process.poll() is not None:
        return
    os.killpg(process.pid, signal.SIGINT)
    try:
        process.wait(timeout=5.)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGKILL)
        process.wait(timeout=5.)


def cell(grid, x, y):
    ox, oy = grid.info.origin.position.x, grid.info.origin.position.y
    column = math.floor((x - ox) / grid.info.resolution)
    row = math.floor((y - oy) / grid.info.resolution)
    if 0 <= column < grid.info.width and 0 <= row < grid.info.height:
        return grid.data[row * grid.info.width + column]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', type=Path, help='New directory for TEST_ONLY ROS logs')
    output = parser.parse_args().output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    with socket.socket() as listener:
        listener.bind(('127.0.0.1', 0))
        port = listener.getsockname()[1]
    os.environ.update(ROS_MASTER_URI=f'http://127.0.0.1:{port}', ROS_IP='127.0.0.1',
                      ROS_HOME=str(output / 'ros'), ROS_LOG_DIR=str(output / 'roslog'))
    for name in ('ROS_HOSTNAME', 'ROS_NAMESPACE'):
        os.environ.pop(name, None)

    import rosgraph
    import rospkg
    import rospy
    import tf2_ros
    import yaml
    from geometry_msgs.msg import TransformStamped
    from nav_msgs.msg import OccupancyGrid
    from sensor_msgs import point_cloud2
    from sensor_msgs.msg import PointCloud2
    from std_msgs.msg import Header

    with ExitStack() as cleanup:
        def launch(label, argv):
            log = cleanup.enter_context((output / (label + '.log')).open('w'))
            process = subprocess.Popen(argv, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            cleanup.callback(stop, process)
            return process

        master = launch('master', ['roscore', '-p', str(port)])
        wait_for(lambda: master.poll() is None and rosgraph.is_master_online(),
                 'The private ROS Master did not start.')
        rospy.init_node('TEST_ONLY_lidar_voxel', disable_signals=True)
        cleanup.callback(rospy.signal_shutdown, 'TEST_ONLY lidar voxel test ended')

        installed_config = Path(rospkg.RosPack().get_path('ugv_bringup')) / 'config'
        site = installed_config / 'online_runtime/config/slicer_config.yaml'
        rospy.set_param('/slicer_arm_cloud_filter/config_file', str(site))
        rospy.set_param('/slicer_arm_cloud_filter/publish_voxel', True)
        fixture = yaml.safe_load((installed_config / 'lidar_voxel.yaml').read_text())['global_costmap']
        fixture.update(global_frame='map', robot_base_frame='base_link', rolling_window=False,
                       track_unknown_space=True, always_send_full_costmap=True, width=8.0, height=8.0,
                       origin_x=-4., origin_y=-4., resolution=.05, update_frequency=10.,
                       publish_frequency=10., robot_radius=.12)
        rospy.set_param('/voxel_probe/costmap', fixture)

        received = {}
        subscribers = [
            rospy.Subscriber('/voxel_probe/costmap/costmap', OccupancyGrid,
                             lambda msg: received.update(grid=msg), queue_size=1),
            rospy.Subscriber('/cloud_nav_voxel', PointCloud2,
                             lambda msg: received.update(cloud=msg), queue_size=1),
        ]
        sensor = rospy.Publisher('/cloud_registered_body', PointCloud2, queue_size=1)
        static_tf = tf2_ros.StaticTransformBroadcaster()
        transforms = []
        # Distinct body and physical laser origins exercise sensor_frame. A
        # wrong world/body origin cannot clear the chosen z=0.12 m voxel.
        for frame, parent, xyz in (('base_link', 'map', (0., 0., 0.)),
                                   ('body', 'base_link', (.081, .02329, .13588)),
                                   ('laser_link', 'base_link', (.07, .0, .18))):
            transform = TransformStamped()
            transform.header = Header(stamp=rospy.Time.now(), frame_id=parent)
            transform.child_frame_id = frame
            (transform.transform.translation.x, transform.transform.translation.y,
             transform.transform.translation.z) = xyz
            transform.transform.rotation.w = 1.
            transforms.append(transform)
        static_tf.sendTransform(transforms)
        arm = launch('filter', [sys.executable, str(ROOT / 'src/slicer_navigation/scripts/arm_cloud_filter.py')])
        costmap = launch('costmap', ['rosrun', 'costmap_2d', 'costmap_2d_node', '__name:=voxel_probe'])
        wait_for(lambda: sensor.get_num_connections() > 0, 'Production arm cloud filter did not subscribe.')

        def observe(points, duration=1.):
            start = time.monotonic()
            while time.monotonic() - start < duration:
                if arm.poll() is not None or costmap.poll() is not None:
                    raise AssertionError('Filter or costmap exited; see ROS logs.')
                body = [(x - .081, y - .02329, z - .13588) for x, y, z in points]
                sensor.publish(point_cloud2.create_cloud_xyz32(
                    Header(stamp=rospy.Time.now(), frame_id='body'), body))
                time.sleep(.075)

        obstacle = (.725, .025, .12)
        arm_point = (.16, .0, .26)
        chassis_point = (.10, .08, .15)
        side_return = (1.5, -1., .0)
        observe([obstacle, arm_point, chassis_point, side_return], duration=2.)
        filtered = received.get('cloud')
        assert filtered is not None and filtered.width == 2, 'Voxel input retained self returns.'
        assert filtered.header.frame_id == 'body'
        grid = received.get('grid')
        assert grid is not None, 'The real voxel costmap did not publish.'
        assert cell(grid, *obstacle[:2]) == 100, 'Low obstacle was not marked.'
        assert cell(grid, 2.525, .025) == -1, 'Unobserved space was not unknown.'

        observe([side_return])
        assert cell(received['grid'], *obstacle[:2]) == 100, 'Missed angle cleared unseen obstacle.'
        observe([(1.925, .067, .55), side_return])
        assert cell(received['grid'], *obstacle[:2]) == 100, 'High ray cleared a low obstacle.'
        observe([(1.925, .067, .0), side_return])
        assert cell(received['grid'], *obstacle[:2]) == 0, 'Measured ground ray did not clear old obstacle.'
        assert cell(received['grid'], 2.525, .025) == -1, 'The ray cleared beyond its measured extent.'
        for subscriber in subscribers:
            subscriber.unregister()
        print('PASS: real VoxelLayer retained low/temporarily occluded obstacle, ignored high ray, '
              'cleared with measured ground ray, preserved unknown, filtered arm/chassis.')


if __name__ == '__main__':
    main()
