#!/usr/bin/env python3
"""Navigation outlet: retain planned curves, smooth commands, brake before the laser stop line."""
import math
from pathlib import Path
import sys
import threading
import time
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
import rospy
import tf
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from navigation_core import (saturate_twist, slew_arc, compensate_turn, swept_clear,
                             parse_footprint, front_scan_clearance, limit_front_approach)


def main():
    rospy.init_node('slicer_nav_arc_guard')
    p = rospy.get_param('~')
    with open(p['config_file'], encoding='utf-8') as stream:
        config = yaml.safe_load(stream)
        limits = config['navigation']
    front_stop = float(limits['front_stop_distance'])
    brake_linear = float(limits.get('brake_linear', limits['accel_linear']))
    if not math.isfinite(brake_linear) or brake_linear <= 0:
        raise ValueError('brake_linear must be finite and positive')
    if front_stop <= 0:
        raise ValueError('front_stop_distance must be positive')
    mount = config['sensors']['base_to_laser']
    laser_x, laser_y = mount['xyz'][:2]
    laser_yaw = tf.transformations.euler_from_quaternion(mount['quaternion'])[2]
    corridor_half_width = max(config['chassis']['body_width']/2,
                              limits['travel_envelope']['half_width'])+limits['footprint_padding']
    if min(limits[key] for key in ('max_forward', 'max_angular', 'accel_linear', 'accel_angular')) <= 0 or limits['max_reverse'] < 0:
        raise ValueError('Velocity and acceleration limits must be positive')
    with open(p['control_config_file'], encoding='utf-8') as stream:
        control = yaml.safe_load(stream)
    min_angular = float(limits['turn_start_angular'])
    body_control = control.get('control_mode') == 'body_speed'
    if not 0 < min_angular <= limits['max_angular']:
        raise ValueError('Turn-start angular speed exceeds navigation angular cap')
    rospy.loginfo('Body closed-loop: planned angular velocity retained; no open-loop turn boost' if body_control
                  else 'Legacy turn compensation: %.3frad/s floor' % min_angular)
    lock, sources = threading.Lock(), {}
    latest_grid, footprint = [None, -math.inf], [None]
    latest_front = [math.inf, -math.inf]
    listener = tf.TransformListener()
    pub = rospy.Publisher(p['output_topic'], Twist, queue_size=1)
    reason_pub = rospy.Publisher('~hold_reason', String, queue_size=1, latch=True)
    adjustment_pub = rospy.Publisher('~adjustment', String, queue_size=1, latch=True)

    def receive(msg, source):
        value = (msg.linear.x, msg.angular.z)
        valid = all(math.isfinite(x) for x in value)
        with lock:
            sources[source] = (time.monotonic(), value if valid else (0., 0.), valid)

    def costmap(msg):
        if msg.info.resolution <= 0 or len(msg.data) != msg.info.width*msg.info.height:
            return
        q = msg.info.origin.orientation
        grid = dict(width=msg.info.width, height=msg.info.height, data=msg.data,
                    resolution=msg.info.resolution, frame=msg.header.frame_id,
                    origin=(msg.info.origin.position.x, msg.info.origin.position.y,
                            tf.transformations.euler_from_quaternion((q.x,q.y,q.z,q.w))[2]))
        with lock:
            latest_grid[:] = [grid, time.monotonic()]

    def scan(msg):
        if msg.header.frame_id == control['base_frame']:
            pose = (0., 0., 0.)
        elif msg.header.frame_id == config['frames']['laser']:
            pose = (laser_x, laser_y, laser_yaw)
        else:
            rospy.logwarn_throttle(5., 'Front stop: unsupported scan frame %s', msg.header.frame_id)
            return
        distance = front_scan_clearance(msg.ranges, msg.angle_min, msg.angle_increment,
                                         msg.range_min, msg.range_max, pose, laser_x, corridor_half_width)
        age = max(0., (rospy.Time.now()-msg.header.stamp).to_sec()) if msg.header.stamp else 0.
        with lock:
            latest_front[:] = [distance, time.monotonic()-age]

    def changed_path_error(grid, grid_at, now, target, output):
        """Only modified paths need a second check; ordinary TEB commands pass through."""
        try:
            if grid is None or now-grid_at > p['costmap_timeout']:
                return 'compensation needs a fresh local grid'
            stamp = listener.getLatestCommonTime(grid['frame'], control['base_frame'])
            if (rospy.Time.now()-stamp).to_sec() > p['tf_timeout']:
                return 'compensation needs fresh base TF'
            position, quat = listener.lookupTransform(grid['frame'], control['base_frame'], stamp)
            pose = (position[0], position[1], tf.transformations.euler_from_quaternion(quat)[2])
            if footprint[0] is None:
                footprint[0] = parse_footprint(rospy.get_param('/move_base/local_costmap/footprint'))
            horizon = max(p['compensation_horizon'],
                          abs(target[0])/brake_linear+p['command_timeout'],
                          abs(target[1])/limits['accel_angular']+p['command_timeout'])
            if not all(swept_clear(pose, *command, footprint[0], grid, horizon) for command in (target, output)):
                return 'compensated swept footprint occupied/unknown/outside grid'
        except (tf.Exception, ValueError, KeyError) as exc:
            return str(exc)
        return ''

    rospy.Subscriber(p['input_topic'], Twist, receive, callback_args='nav', queue_size=1)
    rospy.Subscriber(p.get('spin_input_topic', '/cmd_vel/patrol_spin_raw'), Twist,
                     receive, callback_args='spin', queue_size=1)
    rospy.Subscriber(p['costmap_topic'], OccupancyGrid, costmap, queue_size=1)
    rospy.Subscriber('/scan', LaserScan, scan, queue_size=1)
    rospy.loginfo('Forward stop line: %.2fm from laser centre', front_stop)
    previous, previous_at, last_reason, last_note = (0., 0.), time.monotonic(), None, None
    try:
        while not rospy.is_shutdown():
            now = time.monotonic()
            with lock:
                nav, spin = sources.get('nav'), sources.get('spin')
                grid, grid_at = latest_grid
                front_distance, front_at = latest_front
            sample = (spin if spin and now-spin[0] <= p['command_timeout']
                      and spin[1] != (0., 0.) else nav)
            reason, note, bounded, target, changed = '', '', (0., 0.), (0., 0.), False
            if sample is None or now-sample[0] > p['command_timeout']:
                reason = 'command missing/stale'
            elif not sample[2]:
                reason = 'non-finite input'
            else:
                bounded = saturate_twist(*sample[1], limits['max_forward'],
                                         limits['max_reverse'], limits['max_angular'])
                target = (bounded if body_control else
                          compensate_turn(*bounded, min_angular, p['turning_linear_window'],
                                          p['angular_intent_threshold']))
                reverse_rejected = sample[1][0] < 0 and limits['max_reverse'] == 0
                changed = target != bounded or reverse_rejected
                if target != bounded:
                    note = 'turn-start floor %.3frad/s' % min_angular
                if reverse_rejected:
                    note += '; negative body vx rejected; angular command retained'
            # Explicit stop and expired commands stop immediately. Nonzero commands
            # retain their curvature under saturation, including zero-radius turns.
            output = slew_arc(previous, target, min(.1, now-previous_at),
                              limits['accel_linear'], limits['accel_angular'])
            if not body_control and changed and target != (0., 0.):
                error = changed_path_error(grid, grid_at, now, target, output)
                if error:
                    fallback = slew_arc(previous, bounded, min(.1, now-previous_at),
                                        limits['accel_linear'], limits['accel_angular'])
                    if target != bounded and not changed_path_error(grid, grid_at, now, bounded, fallback):
                        output = fallback
                        note = 'turn boost blocked by footprint; unboosted angular command retained'
                    else:
                        output, reason = (0., 0.), error
            if output[0] > 0:
                scan_age = max(0., now-front_at)
                if scan_age > p['command_timeout']:
                    output, reason = (0., 0.), 'forward motion needs a fresh scan'
                else:
                    # 保留现有规划与包络检查；这里只把前进速度限制到能在15cm线前停住。
                    delay = scan_age+1./p['publish_hz']+1./config['sensors']['lidar_publish_hz']
                    limited = limit_front_approach(output, previous[0], front_distance,
                                                   front_stop, brake_linear, delay)
                    if limited != output:
                        if limited[0] == 0:
                            reason = 'front obstacle %.3fm from laser; stop line %.3fm' % (front_distance, front_stop)
                        else:
                            note = 'front obstacle %.3fm; braking before %.3fm laser stop line' % (front_distance, front_stop)
                    output = limited
            msg = Twist()
            msg.linear.x, msg.angular.z = output
            pub.publish(msg)
            if reason != last_reason:
                reason_pub.publish(String(data=reason))
                last_reason = reason
            if note != last_note:
                adjustment_pub.publish(String(data=note))
                last_note = note
            if reason:
                rospy.logwarn_throttle(3., 'Navigation compensation held stopped: %s', reason)
            elif note:
                rospy.loginfo_throttle(3., 'Navigation compensation: %s; input=%s output=%s', note, sample[1], output)
            previous, previous_at = output, now
            time.sleep(1. / p['publish_hz'])
    finally:
        pub.publish(Twist())


if __name__ == '__main__':
    try:
        main()
    except (rospy.ROSInterruptException, KeyboardInterrupt):
        pass
