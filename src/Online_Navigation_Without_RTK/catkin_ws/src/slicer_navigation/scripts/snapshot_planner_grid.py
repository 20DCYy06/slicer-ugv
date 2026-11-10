#!/usr/bin/env python3
"""Read-only grid/pose capture for isolated GlobalPlanner reproduction; sends no goals."""
import argparse
from pathlib import Path
import time
import rospy
import tf
from nav_msgs.msg import OccupancyGrid
from tf.transformations import euler_from_quaternion
import yaml


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True)
    parser.add_argument('--file', default='/slicer/catkin_ws/src/slicer_navigation/config/waypoints.yaml')
    args = parser.parse_args()
    rospy.init_node('slicer_grid_snapshot', anonymous=True)
    listener = tf.TransformListener()
    grid = rospy.wait_for_message('/move_base/global_costmap/costmap', OccupancyGrid, timeout=10)
    listener.waitForTransform(grid.header.frame_id, 'base_link', rospy.Time(0), rospy.Duration(5))
    xyz, quat = listener.lookupTransform(grid.header.frame_id, 'base_link', rospy.Time(0))
    with open(args.file, encoding='utf-8') as stream:
        data = yaml.safe_load(stream)
    if data['frame_id'] != grid.header.frame_id:
        raise ValueError('Snapshot requires waypoints in the costmap frame')
    target = data['waypoints'][0]
    origin_q = grid.info.origin.orientation
    if abs(euler_from_quaternion((origin_q.x, origin_q.y, origin_q.z, origin_q.w))[2]) > 1e-6:
        raise ValueError('Offline probe requires axis-aligned global grid')
    header = [grid.info.width, grid.info.height, grid.info.resolution,
              grid.info.origin.position.x, grid.info.origin.position.y,
              xyz[0], xyz[1], euler_from_quaternion(quat)[2],
              target['x'], target['y'], target.get('yaw', 0.)]
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(' '.join(map(str, header))+'\n'+' '.join(map(str, grid.data))+'\n')
    print('READ_ONLY_GRID_SNAPSHOT', str(output), 'start=', header[5:8], 'goal=', header[8:])


if __name__ == '__main__':
    main()
