#!/usr/bin/env python3
"""Navigation velocity smoothing with swept-checked turn compensation."""
import math
from pathlib import Path
import sys
import threading
import time
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
import rospy
import tf
from actionlib_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path as RosPath
from std_msgs.msg import String, Time
from navigation_core import (body_twist_between, compensate_turn, footprint_clear, guarded_forward_twist,
                             parse_footprint, planner_limits, saturate_twist, slew_arc, swept_clear)


SWEPT_STOP = 'swept footprint occupied/unknown/outside grid'


def main():
    rospy.init_node('slicer_nav_arc_guard')
    p = rospy.get_param('~')
    lio_timeout = float(p.get('lio_timeout', .5))
    if not math.isfinite(lio_timeout) or lio_timeout <= 0:
        raise ValueError('lio_timeout must be finite and positive')
    allow_patrol_spin = p.get('allow_patrol_spin', True)
    if type(allow_patrol_spin) is not bool:
        raise ValueError('allow_patrol_spin must be a boolean')
    registration_reverse = p.get('registration_reverse', False)
    if type(registration_reverse) is not bool:
        raise ValueError('registration_reverse must be a boolean')
    external_reverse = p.get('external_reverse', False)
    if type(external_reverse) is not bool:
        raise ValueError('external_reverse must be a boolean')
    adaptive_forward_speed = p.get('adaptive_forward_speed', False)
    if type(adaptive_forward_speed) is not bool:
        raise ValueError('adaptive_forward_speed must be a boolean')
    if external_reverse:
        planner_output = p.get('planner_output_topic', p['input_topic'])
        if not all(isinstance(topic, str) and topic.startswith('/')
                   for topic in (planner_output, p['input_topic'])):
            raise ValueError('external reverse requires absolute planner and Guard input topics')
        if rospy.resolve_name(planner_output) == rospy.resolve_name(p['input_topic']):
            raise ValueError('external reverse requires planner output isolated from Guard input')
    with open(p['config_file'], encoding='utf-8') as stream:
        limits = yaml.safe_load(stream)['navigation']
    registration_reverse_cap = min(.8, float(limits['max_forward'])) if registration_reverse else 0.
    planner_ns = str(p.get('planner_namespace', '/move_base')).rstrip('/')
    if not planner_ns.startswith('/') or '..' in planner_ns.split('/'):
        raise ValueError('explicit absolute planner namespace required')
    plugin = rospy.get_param(planner_ns + '/base_local_planner')
    if not isinstance(plugin, str):
        raise ValueError('base_local_planner must name a supported plugin')
    planner = rospy.get_param(planner_ns + '/' + plugin.rsplit('/', 1)[-1], {})
    # Source physics and the selected planner both constrain this outlet. A
    # commissioning planner may deliberately request lower limits than daily nav.
    limits = planner_limits(limits, plugin, planner, external_reverse=external_reverse)
    brake_linear = float(limits.get('brake_linear', limits['accel_linear']))
    if not math.isfinite(brake_linear) or brake_linear <= 0:
        raise ValueError('brake_linear must be finite and positive')
    brake_linear = min(brake_linear, limits['accel_linear'])
    rospy.set_param('~effective_max_reverse', limits['max_reverse'])
    rospy.set_param('~effective_registration_reverse', registration_reverse_cap)
    return_limits = dict(limits, accel_linear=1.)
    rospy.loginfo('Guard planner %s limits: forward=%.3f reverse=%.3f angular=%.3f',
                  plugin, limits['max_forward'], limits['max_reverse'], limits['max_angular'])
    with open(p['control_config_file'], encoding='utf-8') as stream:
        control = yaml.safe_load(stream)
    base = rospy.get_param(planner_ns + '/local_costmap/robot_base_frame', control['base_frame'])
    compensation_enabled = p.get('turn_compensation_enabled', True)
    if type(compensation_enabled) is not bool:
        raise ValueError('turn_compensation_enabled must be a boolean')
    start_pwm, min_angular = 0., 0.
    if compensation_enabled:
        start_pwm = float(limits['turn_start_pwm_permille'])
        gain = float(control['turn_gain_permille_per_rad_s'])
        if not math.isfinite(gain) or gain <= 0:
            raise ValueError('Invalid turn PWM gain')
        min_angular = start_pwm / gain
        if not (0 < start_pwm <= control['ros_driver_pwm_limit_permille'] and
                min_angular <= limits['max_angular']):
            raise ValueError('Turn-start PWM exceeds effective planner angular or driver cap')
        rospy.loginfo('Turn compensation: %.0f permille = %.3frad/s floor; reverse limit=%.3fm/s',
                      start_pwm, min_angular, limits['max_reverse'])
    lock, sources = threading.Lock(), {}
    latest_grid, footprint = [None, -math.inf], [None]
    latest_lio = [-math.inf, None, '', None, None]
    action_status = [None]
    listener = tf.TransformListener()
    pub = rospy.Publisher(p['output_topic'], Twist, queue_size=1)
    reason_pub = rospy.Publisher('~hold_reason', String, queue_size=1, latch=True)
    adjustment_pub = rospy.Publisher('~adjustment', String, queue_size=1, latch=True)
    committed = [None]
    commitment_stop = None
    if planner.get('start_commitment') is True:
        commitment_stop = rospy.Publisher('~commitment_stop', Time, queue_size=1)

    def commitment_path(message):
        if message._connection_header.get('callerid') != planner_ns:
            return
        poses = []
        for item in message.poses[:64]:
            p, q = item.pose.position, item.pose.orientation
            values = (p.x, p.y, q.z, q.w)
            if not all(math.isfinite(v) for v in values):
                return
            poses.append((p.x, p.y, 2. * math.atan2(q.z, q.w)))
        committed[0] = message.header.frame_id, message.header.stamp, poses

    def committed_block(grid, grid_at, now):
        sample = committed[0]
        if sample is None or grid is None or now-grid_at > p['costmap_timeout']:
            return rospy.Time()
        frame, stamp, poses = sample
        if frame != grid['frame'] or not 0. <= (rospy.Time.now()-stamp).to_sec() <= .35:
            return rospy.Time()
        if footprint[0] is None:
            try:
                footprint[0] = parse_footprint(rospy.get_param(planner_ns + '/local_costmap/footprint'))
            except (KeyError, TypeError, ValueError):
                return rospy.Time()
        # Only a measured lethal cell can justify abandoning the locked side.
        # Other Guard tests still stop for unknown cells and map boundaries.
        for pose in poses:
            if not footprint_clear(pose, footprint[0], grid, unknown=False, outside_is_blocked=False):
                return stamp
        return rospy.Time()


    def receive(msg, source):
        value = (msg.linear.x, msg.angular.z)
        valid = all(math.isfinite(x) for x in value)
        with lock:
            sources[source] = (time.monotonic(), value if valid else (0., 0.), valid)

    def registration_command(msg):
        header = getattr(msg, '_connection_header', {}) or {}
        if (header.get('callerid') != '/slicer_geographic_roundtrip'
                or header.get('latching', '0') != '0'):
            return
        v, w = msg.linear.x, msg.angular.z
        valid = (-registration_reverse_cap <= v < 0. and -.15 <= w <= .15) or (v == 0. and w == 0.)
        valid = valid and all(x == 0. for x in (msg.linear.y, msg.linear.z, msg.angular.x, msg.angular.y))
        with lock:
            sources['registration_reverse'] = time.monotonic(), (v, w) if valid else (0., 0.), valid

    def move_base_status(msg):
        header = getattr(msg, '_connection_header', {}) or {}
        # actionlib publishes latched status periodically. Check its origin
        # here and its ROS stamp and receipt age at every use.
        if header.get('callerid') != planner_ns:
            return
        active = any(item.status in (GoalStatus.PENDING, GoalStatus.ACTIVE,
                                    GoalStatus.PREEMPTING, GoalStatus.RECALLING)
                     for item in msg.status_list)
        with lock:
            action_status[0] = time.monotonic(), msg.header.stamp, active

    def lio(msg):
        received, stamp = time.monotonic(), msg.header.stamp
        age = (rospy.Time.now()-stamp).to_sec()
        if stamp.to_sec() <= 0 or not math.isfinite(age) or age < 0 or age > lio_timeout:
            return
        pose = None
        if (adaptive_forward_speed or registration_reverse) and msg.header.frame_id and msg.child_frame_id == base:
            position, q = msg.pose.pose.position, msg.pose.pose.orientation
            quaternion = (q.x, q.y, q.z, q.w)
            if all(math.isfinite(a) for a in (position.x, position.y, *quaternion)) and math.isclose(
                    math.hypot(*quaternion), 1., rel_tol=1e-3):
                pose = (position.x, position.y, tf.transformations.euler_from_quaternion(quaternion)[2])
        with lock:
            # Repeated old headers must not keep a stopped LIO source alive.
            _, prev_stamp, prev_frame, prev_pose, _ = latest_lio
            if prev_stamp is not None and stamp <= prev_stamp:
                return
            observed = None
            if pose is not None and prev_pose is not None and prev_frame == msg.header.frame_id:
                elapsed = (stamp-prev_stamp).to_sec()
                if elapsed <= lio_timeout:
                    # The source may publish an unpopulated Odometry.twist.
                    vx, vy, wz = body_twist_between(prev_pose, pose, elapsed)
                    observation_limit = 1.6 if registration_reverse else limits['max_forward']
                    if math.hypot(vx, vy) <= observation_limit and abs(wz) <= limits['max_angular']:
                        observed = (vx, vy, wz)
            latest_lio[:] = [received, stamp, msg.header.frame_id, pose, observed]

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

    def check_motion(grid, grid_at, now, target, output, previous, dt, returning=False):
        """Every nonzero output requires fresh context and a clear swept footprint."""
        try:
            with lock:
                lio_received, lio_stamp, _, _, observed = latest_lio
            if now-lio_received > lio_timeout:
                return 'motion needs fresh LIO odometry', None
            lio_age = (rospy.Time.now()-lio_stamp).to_sec()
            if lio_age < 0 or lio_age > lio_timeout:
                return 'motion needs a current advancing LIO stamp', None
            if grid is None or now-grid_at > p['costmap_timeout']:
                return 'motion needs a fresh local grid', None
            stamp = listener.getLatestCommonTime(grid['frame'], base)
            if (rospy.Time.now()-stamp).to_sec() > p['tf_timeout']:
                return 'motion needs fresh base TF', None
            position, quat = listener.lookupTransform(grid['frame'], base, stamp)
            pose = (position[0], position[1], tf.transformations.euler_from_quaternion(quat)[2])
            if footprint[0] is None:
                footprint[0] = parse_footprint(rospy.get_param(planner_ns + '/local_costmap/footprint'))
            horizon = max(p['compensation_horizon'],
                          abs(target[0])/brake_linear+p['command_timeout'],
                          abs(target[1])/limits['accel_angular']+p['command_timeout'])
            if returning:
                if observed is None:
                    return 'registration return needs fresh LIO motion', None
                vx, vy, wz = observed
                angular_accel = limits['accel_angular']
                for v, lateral, w in ((vx, vy, wz), (previous[0], 0., previous[1])):
                    stopping_horizon = max(p['compensation_horizon'],
                                           math.hypot(v, lateral)/brake_linear + p['command_timeout'],
                                           abs(w)/angular_accel + p['command_timeout'])
                    if not swept_clear(pose, v, w, footprint[0], grid, stopping_horizon, lateral=lateral):
                        return SWEPT_STOP, None
                # Reuse the adaptive braking check along the reverse heading.
                # Rotate the footprint as well to preserve its asymmetric overhang.
                reverse_pose = (pose[0], pose[1], pose[2] + math.pi)
                reverse_footprint = tuple((-x, -y) for x, y in footprint[0])
                guarded = guarded_forward_twist(
                    reverse_pose, (-target[0], target[1]), (-output[0], output[1]),
                    (-previous[0], previous[1]), (-vx, -vy, wz), reverse_footprint,
                    grid, p['compensation_horizon'], brake_linear, p['command_timeout'],
                    dt, return_limits, recovering=swept_held)
                if guarded is not None:
                    return '', (-guarded[0], guarded[1])
            elif (adaptive_forward_speed and not compensation_enabled
                    and (swept_held or target[0] > 0. or (target[0] == 0. and previous[0] > 0.))):
                guarded = guarded_forward_twist(pose, target, output, previous, observed,
                                                footprint[0], grid, p['compensation_horizon'],
                                                brake_linear, p['command_timeout'], dt, limits,
                                                recovering=swept_held)
                if guarded is not None:
                    return '', guarded if guarded != output else None
            elif all(swept_clear(pose, *command, footprint[0], grid, horizon) for command in (target, output)):
                return '', None
            return SWEPT_STOP, None
        except (tf.Exception, ValueError, KeyError) as exc:
            return str(exc), None

    rospy.Subscriber(p.get('lio_topic', '/lio/odom'), Odometry, lio, queue_size=1)
    rospy.Subscriber(p['input_topic'], Twist, receive, callback_args='nav', queue_size=1)
    if allow_patrol_spin:
        rospy.Subscriber(p.get('spin_input_topic', '/cmd_vel/patrol_spin_raw'), Twist,
                         receive, callback_args='spin', queue_size=1)
    rospy.Subscriber(p['costmap_topic'], OccupancyGrid, costmap, queue_size=1)
    if commitment_stop is not None:
        rospy.Subscriber(planner_ns + '/' + plugin.rsplit('/', 1)[-1] + '/commitment_path',
                         RosPath, commitment_path, queue_size=1)

    if registration_reverse:
        if planner_ns != '/ugv/move_base' or external_reverse:
            raise ValueError('registration reverse requires the standard RTK preparation Guard')
        rospy.Subscriber('/geographic/registration_reverse', Twist, registration_command, queue_size=1)
        rospy.Subscriber(planner_ns + '/status', GoalStatusArray, move_base_status, queue_size=1)
    previous, previous_at, last_reason, last_note = (0., 0.), time.monotonic(), None, None
    was_returning = swept_held = False
    period = 1. / p['publish_hz']
    try:
        while not rospy.is_shutdown():
            now = time.monotonic()
            with lock:
                nav, spin = sources.get('nav'), sources.get('spin')
                reverse, action = sources.get('registration_reverse'), action_status[0]
                grid, grid_at = latest_grid
            returning = reverse is not None and now-reverse[0] <= min(.25, p['command_timeout'])
            if returning != was_returning:
                previous = (0., 0.)
                was_returning = returning
            sample = (spin if spin and now-spin[0] <= p['command_timeout']
                      and spin[1] != (0., 0.) else nav)
            if returning:
                # Suppress both sources on a contention or stale status. The
                # reverse command expires independently if its client exits.
                sample = reverse
            reason, note, bounded, target = '', '', (0., 0.), (0., 0.)
            reverse_without_idle = returning and (
                action is None or action[2] or action[1].to_sec() <= 0.
                or now-action[0] > .6 or not -.1 <= (rospy.Time.now()-action[1]).to_sec() <= .6
                or (nav and now-nav[0] <= p['command_timeout'] and nav[1] != (0., 0.))
                or (spin and now-spin[0] <= p['command_timeout'] and spin[1] != (0., 0.)))
            reverse_cap = registration_reverse_cap if returning else limits['max_reverse']
            linear_accel = return_limits['accel_linear'] if returning else limits['accel_linear']
            if reverse_without_idle:
                reason = 'registration return requires fresh idle navigation'
            elif sample is None or now-sample[0] > p['command_timeout']:
                reason = 'command missing/stale'
            elif not sample[2]:
                reason = 'non-finite input'
            elif reverse_cap == 0 and sample[1][0] < 0:
                # A reverse arc is not a pure-spin request. Reject it before any
                # turn floor or collision fallback can preserve its angular part.
                reason = 'negative body vx disallowed; full command stopped'
            else:
                bounded = saturate_twist(*sample[1], limits['max_forward'],
                                         reverse_cap, limits['max_angular'])
                target = (compensate_turn(*bounded, min_angular, p['turning_linear_window'],
                                          p['angular_intent_threshold'])
                          if compensation_enabled and not returning else bounded)
                if target != bounded:
                    note = 'turn-start floor %.0f permille (%.3frad/s)' % (start_pwm, min_angular)
            # Explicit stop and expired commands stop immediately. Nonzero commands
            # retain their curvature under saturation, including zero-radius turns.
            dt = min(.1, now-previous_at)
            output = slew_arc(previous, target, dt,
                              linear_accel, limits['accel_angular'])
            if target != (0., 0.) or output != (0., 0.):
                error, reduced = check_motion(grid, grid_at, now, target, output, previous, dt, returning)
                if error:
                    fallback = slew_arc(previous, bounded, dt,
                                        limits['accel_linear'], limits['accel_angular'])
                    if target != bounded and not check_motion(grid, grid_at, now, bounded, fallback, previous, dt)[0]:
                        output = fallback
                        note = 'turn boost blocked by footprint; unboosted angular command retained'
                    else:
                        output, reason = (0., 0.), error
                    if error == SWEPT_STOP:
                        swept_held = True
                else:
                    swept_held = False
                    if reduced is not None and reduced != output:
                        output = reduced
                        direction = 'registration reverse' if returning else 'forward'
                        note = direction + ' speed reduced within observed braking envelope'
            if commitment_stop is not None:
                stopped_path = committed_block(grid, grid_at, now)
                if not stopped_path.is_zero():
                    output, reason = (0., 0.), 'committed path has a hard obstacle within 1.2 m'
                commitment_stop.publish(Time(data=stopped_path))
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
            time.sleep(max(0., period - (time.monotonic() - now)))
    finally:
        pub.publish(Twist())


if __name__ == '__main__':
    try:
        main()
    except (rospy.ROSInterruptException, KeyboardInterrupt):
        pass
