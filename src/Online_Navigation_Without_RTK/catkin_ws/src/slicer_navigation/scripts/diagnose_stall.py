#!/usr/bin/env python3
"""Read-only snapshot of the move_base planning and velocity chain."""
import argparse
from collections import deque
import math
import time
from pathlib import Path as FilePath
import sys

import rospy
import tf
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Path
from nav_msgs.srv import GetPlan
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion, quaternion_from_euler
sys.path.insert(0, str(FilePath(__file__).resolve().parent))
from navigation_core import footprint_clear, parse_footprint


def cell(grid, x, y):
    origin = grid.info.origin
    q = origin.orientation
    yaw = euler_from_quaternion((q.x, q.y, q.z, q.w))[2]
    dx, dy = x-origin.position.x, y-origin.position.y
    gx = math.cos(yaw)*dx + math.sin(yaw)*dy
    gy = -math.sin(yaw)*dx + math.cos(yaw)*dy
    ix, iy = int(math.floor(gx/grid.info.resolution)), int(math.floor(gy/grid.info.resolution))
    if ix < 0 or iy < 0 or ix >= grid.info.width or iy >= grid.info.height:
        return 'outside'
    return grid.data[iy*grid.info.width+ix]


def connectivity(grid, sx, sy, gx, gy):
    """Read-only 4-neighbor check; distinguishes blocked topology from planner extraction."""
    origin = grid.info.origin
    q = origin.orientation
    yaw = euler_from_quaternion((q.x, q.y, q.z, q.w))[2]
    def index(x, y):
        dx, dy = x-origin.position.x, y-origin.position.y
        mx = math.cos(yaw)*dx + math.sin(yaw)*dy
        my = -math.sin(yaw)*dx + math.cos(yaw)*dy
        return int(math.floor(mx/grid.info.resolution)), int(math.floor(my/grid.info.resolution))
    start, goal = index(sx, sy), index(gx, gy)
    width, height, data = grid.info.width, grid.info.height, grid.data
    def free(x, y):
        return 0 <= x < width and 0 <= y < height and 0 <= data[y*width+x] < 100
    if not free(*start) or not free(*goal):
        return False, 0
    queue, seen = deque([start]), {start}
    while queue:
        x, y = queue.popleft()
        if (x, y) == goal:
            return True, len(seen)
        for neighbor in ((x-1,y),(x+1,y),(x,y-1),(x,y+1)):
            if neighbor not in seen and free(*neighbor):
                seen.add(neighbor)
                queue.append(neighbor)
    return False, len(seen)


def audit_path(grid, path, label):
    frame = path.header.frame_id or (path.poses[0].header.frame_id if path.poses else '')
    if frame != grid.header.frame_id:
        print('%s: frame mismatch %s != %s' %
              (label, frame, grid.header.frame_id), flush=True)
        return
    values = [cell(grid, pose.pose.position.x, pose.pose.position.y)
              for pose in path.poses]
    lethal = sum(isinstance(value, int) and value >= 100 for value in values)
    unknown = sum(value == -1 or value == 'outside' for value in values)
    print('%s: poses=%d lethal=%d unknown/outside=%d '
          '(center cells only; path/costmap timestamps may differ)' %
          (label, len(values), lethal, unknown), flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--x', type=float, required=True)
    parser.add_argument('--y', type=float, required=True)
    parser.add_argument('--yaw', type=float, default=0.0)
    parser.add_argument('--duration', type=float, default=3.0)
    args = parser.parse_args()
    rospy.init_node('slicer_stall_diagnostic', anonymous=True)
    listener = tf.TransformListener()
    latest = {}

    def watch(topic, kind):
        def callback(msg):
            latest[topic] = (time.monotonic(), msg)
        rospy.Subscriber(topic, kind, callback, queue_size=1)

    for topic in ('/cmd_vel/nav_raw', '/cmd_vel/nav', '/cmd_vel'):
        watch(topic, Twist)
    for topic in ('/move_base/GlobalPlanner/plan', '/move_base/TebLocalPlannerROS/local_plan'):
        watch(topic, Path)
    for topic in ('/map', '/move_base/global_costmap/costmap', '/move_base/local_costmap/costmap'):
        watch(topic, OccupancyGrid)
    watch('/slicer_nav_arc_guard/hold_reason', String)

    deadline = time.monotonic() + max(0.5, args.duration)
    while not rospy.is_shutdown() and time.monotonic() < deadline:
        time.sleep(.1)
    now = time.monotonic()
    for topic, entry in latest.items():
        at, msg = entry
        detail = ('vx=%.3f wz=%.3f' % (msg.linear.x, msg.angular.z) if isinstance(msg, Twist)
                  else 'poses=%d' % len(msg.poses) if isinstance(msg, Path)
                  else 'reason=%r' % msg.data if isinstance(msg, String)
                  else 'grid=%dx%d' % (msg.info.width, msg.info.height))
        print('%s age=%.2fs %s' % (topic, now-at, detail), flush=True)
    for topic in ('/cmd_vel/nav_raw', '/cmd_vel/nav', '/cmd_vel'):
        if topic not in latest:
            print('%s: no message during observation window' % topic, flush=True)

    try:
        listener.waitForTransform('map', 'base_link', rospy.Time(0), rospy.Duration(2))
        pos, quat = listener.lookupTransform('map', 'base_link', rospy.Time(0))
        print('robot map XY=(%.3f, %.3f), goal=(%.3f, %.3f)' %
              (pos[0], pos[1], args.x, args.y), flush=True)
    except tf.Exception as exc:
        print('map->base_link unavailable: %s' % exc, flush=True)
        return
    local_entry = latest.get('/move_base/local_costmap/costmap')
    if local_entry:
        local_grid = local_entry[1]
        try:
            local_pos, local_quat = listener.lookupTransform(local_grid.header.frame_id,
                                                             'base_link', rospy.Time(0))
            print('local cost at robot=%s' %
                  cell(local_grid, local_pos[0], local_pos[1]), flush=True)
            footprint = parse_footprint(rospy.get_param('/move_base/local_costmap/footprint'))
            origin = local_grid.info.origin
            grid = dict(width=local_grid.info.width, height=local_grid.info.height,
                        resolution=local_grid.info.resolution, data=local_grid.data,
                        origin=(origin.position.x, origin.position.y,
                                euler_from_quaternion((origin.orientation.x,
                                                       origin.orientation.y,
                                                       origin.orientation.z,
                                                       origin.orientation.w))[2]))
            yaw = euler_from_quaternion(local_quat)[2]
            print('local footprint clear=%s' %
                  footprint_clear((local_pos[0], local_pos[1], yaw), footprint, grid),
                  flush=True)
        except tf.Exception as exc:
            print('local cost lookup unavailable: %s' % exc, flush=True)
    entry = latest.get('/move_base/global_costmap/costmap')
    static_map = latest.get('/map')
    if static_map:
        print('static map at robot=%s goal=%s' %
              (cell(static_map[1], pos[0], pos[1]),
               cell(static_map[1], args.x, args.y)), flush=True)
    if entry:
        grid = entry[1]
        print('global cost at robot=%s goal=%s (100=lethal, -1=unknown)' %
              (cell(grid, pos[0], pos[1]), cell(grid, args.x, args.y)), flush=True)
        connected, count = connectivity(grid, pos[0], pos[1], args.x, args.y)
        print('global free-cell connectivity=%s explored=%d' % (connected, count), flush=True)
        path_entry = latest.get('/move_base/GlobalPlanner/plan')
        if path_entry:
            audit_path(grid, path_entry[1], 'published global plan')

    if not rospy.has_param('/move_base/base_global_planner'):
        return
    rospy.wait_for_service('/move_base/make_plan', timeout=3)
    start, goal = PoseStamped(), PoseStamped()
    start.header.frame_id = goal.header.frame_id = 'map'
    start.header.stamp = goal.header.stamp = rospy.Time.now()
    start.pose.position.x, start.pose.position.y = pos[0], pos[1]
    start.pose.orientation.x, start.pose.orientation.y, start.pose.orientation.z, start.pose.orientation.w = quat
    goal.pose.position.x, goal.pose.position.y = args.x, args.y
    q = quaternion_from_euler(0, 0, args.yaw)
    goal.pose.orientation.x, goal.pose.orientation.y, goal.pose.orientation.z, goal.pose.orientation.w = q
    result = rospy.ServiceProxy('/move_base/make_plan', GetPlan)(start, goal, .0)
    print('make_plan poses=%d' % len(result.plan.poses), flush=True)
    if entry:
        audit_path(entry[1], result.plan, 'fresh make_plan')


if __name__ == '__main__':
    main()
