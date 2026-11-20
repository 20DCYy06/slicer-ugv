#!/usr/bin/env python3
"""Latched XY arrival, measured-heading alignment and bounded backup recovery."""
import math
import json
from collections import deque
from pathlib import Path
import queue
import sys
import threading
import time

sys.path.insert(0, str(Path(__file__).resolve().parent))
import actionlib
import rospy
import tf
from actionlib_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import OccupancyGrid, Path as RosPath
from sensor_msgs.msg import LaserScan
from rosgraph_msgs.msg import Log
from std_msgs.msg import Bool, String
from std_srvs.srv import Empty
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from navigation_core import load_waypoints, motion_delta, is_stalled

TERMINAL = {GoalStatus.PREEMPTED, GoalStatus.SUCCEEDED, GoalStatus.ABORTED,
            GoalStatus.REJECTED, GoalStatus.RECALLED, GoalStatus.LOST}


def angle_error(target, actual):
    return math.atan2(math.sin(target-actual), math.cos(target-actual))


class Patrol:
    def __init__(self):
        self.p = rospy.get_param('~')
        self.data = load_waypoints(self.p['file'])
        if not self.data['waypoints']:
            raise ValueError('No waypoints recorded')
        self.listener = tf.TransformListener()
        self.client = actionlib.SimpleActionClient(self.p['action_name'], MoveBaseAction)
        self.active, self.stopping = False, False
        self.ready, self.localized, self.ready_at = False, False, -math.inf
        self.scan_at, self.costmap_at = -math.inf, -math.inf
        self.speed_topics, self.plan_topics, self.guard_reason = {}, {}, ''
        self.speed_stats, self.speed_subscribers = {}, {}
        self.planner_events = deque(maxlen=8)
        self.status_at = -math.inf
        self.state_pub = rospy.Publisher('~state', String, queue_size=1, latch=True)
        self.done_pub = rospy.Publisher('~completed', Bool, queue_size=1, latch=True)
        self.stop_pub = rospy.Publisher(self.p.get('stop_topic', '/cmd_vel/mission'), Twist, queue_size=1)
        self.spin_pub = rospy.Publisher(self.p['spin_topic'], Twist, queue_size=1)
        self.max_angular = rospy.get_param('/move_base/TebLocalPlannerROS/max_vel_theta')
        rospy.Subscriber(self.p['control_ready_topic'], Bool, self.on_ready, queue_size=1)
        rospy.Subscriber(self.p['localization_topic'], Bool, self.on_localization, queue_size=1)
        for topic in ('/cmd_vel/nav_raw', '/cmd_vel/nav', '/cmd_vel'):
            self.speed_subscribers[topic] = rospy.Subscriber(topic, Twist, self.on_speed,
                                                            callback_args=topic, queue_size=1)
        for topic in ('/move_base/GlobalPlanner/plan', '/move_base/TebLocalPlannerROS/local_plan'):
            rospy.Subscriber(topic, RosPath, self.on_plan, callback_args=topic, queue_size=1)
        rospy.Subscriber('/scan', LaserScan, self.on_scan, queue_size=1)
        rospy.Subscriber('/move_base/local_costmap/costmap', OccupancyGrid,
                         self.on_costmap, queue_size=1)
        rospy.Subscriber('/slicer_nav_arc_guard/hold_reason', String,
                         self.on_guard_reason, queue_size=1)
        rospy.Subscriber(self.p['action_name'] + '/status', GoalStatusArray, self.on_status, queue_size=1)
        rospy.Subscriber('/rosout_agg', Log, self.on_planner_log, queue_size=20)
        self.done_pub.publish(False)
        rospy.on_shutdown(self.shutdown)

    def on_ready(self, msg):
        self.ready, self.ready_at = msg.data, time.monotonic()

    def on_localization(self, msg):
        self.localized = msg.data

    def on_speed(self, msg, topic):
        now = time.monotonic()
        previous = self.speed_topics.get(topic)
        count, gap = self.speed_stats.get(topic, (0, 0.))
        self.speed_stats[topic] = (count+1, max(gap, now-previous[0] if previous else 0.))
        self.speed_topics[topic] = (now, msg.linear.x, msg.angular.z)

    def on_status(self, msg):
        self.status_at = time.monotonic()

    def on_planner_log(self, msg):
        if msg.name == '/move_base' and msg.level >= Log.WARN:
            self.planner_events.append((time.monotonic(), msg.msg))
            rospy.logwarn_throttle(3., '[Patrol] MOVE_BASE: %s', msg.msg)

    def on_plan(self, msg, topic):
        self.plan_topics[topic] = (time.monotonic(), len(msg.poses))

    def on_scan(self, msg):
        self.scan_at = time.monotonic()

    def on_costmap(self, msg):
        self.costmap_at = time.monotonic()

    def on_guard_reason(self, msg):
        self.guard_reason = msg.data

    def running(self):
        return not self.stopping and not rospy.is_shutdown()

    def state(self, message):
        self.state_pub.publish(message)
        rospy.loginfo('[Patrol] %s', message)

    def chassis_ready(self):
        return self.ready and time.monotonic()-self.ready_at < self.p['control_ready_timeout']

    def pose(self):
        try:
            frame = self.data['frame_id']
            stamp = self.listener.getLatestCommonTime(frame, self.p['base_frame'])
            if not 0 <= (rospy.Time.now()-stamp).to_sec() <= self.p['pose_timeout']:
                return None
            xyz, quat = self.listener.lookupTransform(frame, self.p['base_frame'], stamp)
            return xyz[0], xyz[1], euler_from_quaternion(quat)[2]
        except tf.Exception:
            return None

    def wait_initial(self):
        self.state('WAITING_FOR_CHASSIS')
        deadline = time.monotonic()+self.p['control_startup_timeout']
        while self.running() and not self.chassis_ready() and time.monotonic() < deadline:
            time.sleep(.1)
        if not self.chassis_ready():
            raise RuntimeError('C30D chassis not ready')
        self.state('WAITING_FOR_MOVE_BASE')
        if not self.client.wait_for_server(rospy.Duration(self.p['server_timeout'])):
            raise RuntimeError('move_base unavailable')
        self.state('WAITING_FOR_LOCALIZATION')
        deadline = time.monotonic()+self.p['localization_timeout']
        while (self.running() and self.p['wait_for_localization'] and
               not self.localized and time.monotonic() < deadline):
            time.sleep(.1)
        if self.p['wait_for_localization'] and not self.localized:
            raise RuntimeError('Localization unavailable')
        self.state('WAITING_FOR_TF')
        deadline = time.monotonic()+self.p['startup_tf_timeout']
        while self.running() and self.pose() is None and time.monotonic() < deadline:
            time.sleep(.05)
        if self.pose() is None:
            raise RuntimeError('Fresh map to base TF unavailable')
        self.state('TF_READY')

    def goal(self, point):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = self.data['frame_id']
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = point['x']
        goal.target_pose.pose.position.y = point['y']
        goal.target_pose.pose.position.z = point.get('z', 0.)
        yaw = point.get('yaw')
        if yaw is None:
            pose = self.pose()
            yaw = math.atan2(point['y']-pose[1], point['x']-pose[0]) if pose else 0.
        quat = quaternion_from_euler(0, 0, yaw)
        orientation = goal.target_pose.pose.orientation
        orientation.x, orientation.y, orientation.z, orientation.w = quat
        return goal

    def cancel_active_goal(self):
        """Do not cancel an XY goal that has already ended during yaw alignment."""
        if not self.active:
            return
        if self.client.get_state() in TERMINAL:
            self.active = False
            return
        self.client.cancel_goal()

    def shutdown(self):
        self.stopping = True
        self.cancel_active_goal()
        self.active = False
        try:
            self.spin_pub.publish(Twist())
            self.stop_pub.publish(Twist())
        except rospy.ROSException:
            pass

    def diagnostic(self, event):
        now = time.monotonic()
        def age(stamp):
            return round(now-stamp, 3) if math.isfinite(stamp) else None
        speeds = {}
        for topic, subscriber in self.speed_subscribers.items():
            sample = self.speed_topics.get(topic)
            count, gap = self.speed_stats.get(topic, (0, 0.))
            speeds[topic] = dict(age_s=age(sample[0]) if sample else None,
                                last=list(sample[1:]) if sample else None,
                                messages=count, max_gap_s=round(gap, 3),
                                connections=subscriber.get_num_connections())
        events = list(self.planner_events)
        snapshot = dict(event=event, state=self.client.get_state(),
                        status_text=self.client.get_goal_status_text(), status_age_s=age(self.status_at),
                        scan_age_s=age(self.scan_at), costmap_age_s=age(self.costmap_at),
                        speeds=speeds, reason=self.guard_reason,
                        plans={topic: self.plan_diagnostic(topic, now)
                               for topic in ('/move_base/GlobalPlanner/plan',
                                             '/move_base/TebLocalPlannerROS/local_plan')},
                        planner_events=[dict(age_s=age(stamp), text=text) for stamp,text in events])
        rospy.logwarn('[Patrol] DIAG %s', json.dumps(snapshot, ensure_ascii=False))

    def plan_diagnostic(self, topic, now):
        """Show last length AND age; stale data is not an observed empty path."""
        value = self.plan_topics.get(topic)
        return dict(poses=value[1] if value else None,
                    age_s=round(now-value[0], 3) if value else None,
                    fresh=bool(value and now-value[0] < self.p['stall_time']))

    def plan_count(self, topic, now):
        # Existing recovery freshness policy, intentionally unchanged.
        value = self.plan_topics.get(topic)
        return value[1] if value and now-value[0] < self.p['stall_time'] else 0

    def stall_report(self, point, pose, anchor=None, elapsed=0.):
        now = time.monotonic()
        global_plan = self.plan_diagnostic('/move_base/GlobalPlanner/plan', now)
        local_plan = self.plan_diagnostic('/move_base/TebLocalPlannerROS/local_plan', now)
        def speed(topic):
            value = self.speed_topics.get(topic)
            return 'none' if not value or now-value[0] > 1. else '%.3f/%.3f' % value[1:]
        rospy.logwarn('[Patrol] STALL point=%s xy_error=%.3fm state=%d raw=%s '
                      'smooth=%s cmd_vel=%s global_plan=%s local_plan=%s '
                      'global_age_s=%s local_age_s=%s local_fresh=%s reason=%s',
                      point['name'], math.hypot(point['x']-pose[0], point['y']-pose[1]),
                      self.client.get_state(), speed('/cmd_vel/nav_raw'),
                      speed('/cmd_vel/nav'), speed('/cmd_vel'),
                      global_plan['poses'], local_plan['poses'],
                      global_plan['age_s'], local_plan['age_s'], local_plan['fresh'],
                      self.guard_reason or 'none')
        self.diagnostic('STALL ' + point['name'])
        if anchor is not None:
            distance, yaw = motion_delta(anchor, pose)
            rospy.logwarn('[Patrol] NO_PROGRESS %.2fs d_xy=%.4fm d_yaw=%.4frad '
                          '(both below %.3fm / %.3frad); commanded turn alone is not measured rotation',
                          elapsed, distance, yaw, self.p['stall_distance'], self.p['stall_yaw'])

    def clear_async(self):
        result = queue.Queue(maxsize=1)
        def worker():
            try:
                rospy.wait_for_service('/move_base/clear_costmaps', timeout=2.)
                rospy.ServiceProxy('/move_base/clear_costmaps', Empty)()
                result.put((True, time.monotonic(), ''))
            except (rospy.ROSException, rospy.ServiceException) as exc:
                result.put((False, time.monotonic(), str(exc)))
        threading.Thread(target=worker, daemon=True).start()
        return result

    def align_heading(self, target, measured_yaw, tolerance):
        """Use map/base heading from the existing IMU/LIO/EKF chain, not a timer.

        The raw Livox IMU has angular rates, not a map-referenced yaw. Keep the
        established fused TF reference and the existing angular limit/deadband.
        """
        error = angle_error(target, measured_yaw)
        command = Twist()
        aligned = abs(error) < tolerance
        if not aligned:
            command.angular.z = math.copysign(min(self.max_angular, abs(error)), error)
        self.spin_pub.publish(command)
        return aligned, error

    def begin_recovery(self, point, attempts, maximum):
        """Use the same cancel -> backup sequence for driving or a blocked turn."""
        self.cancel_active_goal()
        self.spin_pub.publish(Twist())
        self.stop_pub.publish(Twist())
        if attempts < maximum:
            attempts += 1
            self.state('RECOVERING %s attempt=%d' % (point['name'], attempts))
            return 'CANCEL', attempts
        return 'SKIP_CANCEL', attempts

    def navigate(self, point, align_departure=False):
        """XY arrival is latched once; yaw alignment never re-opens the XY goal."""
        deadline = time.monotonic()+self.p['goal_timeout']
        mode = 'DEPART_YAW' if align_departure else 'SEND'
        result, cleared_at = None, None
        mode_at, pause_at = time.monotonic(), None
        anchor, anchor_at = None, time.monotonic()
        retries, last_stall, last_cancel_notice = 0, -math.inf, -math.inf
        max_backups = self.p.get('max_backup_attempts', self.p.get('max_retries', 2))
        if align_departure:
            self.state('ALIGN_DEPARTURE ' + point['name'])
        while self.running():
            now, pose = time.monotonic(), self.pose()
            healthy = (self.chassis_ready() and pose is not None and
                       (not self.p['wait_for_localization'] or self.localized))
            if not healthy:
                self.spin_pub.publish(Twist())
                self.stop_pub.publish(Twist())
                if pause_at is None:
                    self.cancel_active_goal()
                    # An interrupted timed backup ends here; never restart its
                    # full duration after localization/chassis recovery.
                    if mode == 'BACKUP':
                        mode, mode_at = 'BACKUP_STOP', now
                    pause_at = now
                    self.state('PAUSED ' + point['name'])
                if self.p.get('wait_for_chassis_recovery', False) and not self.chassis_ready():
                    # 底盘故障在零速等待，不消耗定位恢复的15秒；定位本身仍用原超时。
                    pause_at = now
                if now-pause_at > self.p['localization_pause_timeout']:
                    rospy.logerr('[Patrol] Localization/chassis did not recover while stopped')
                    return False
                time.sleep(.05)
                continue
            if pause_at is not None:
                if self.p.get('wait_for_chassis_recovery', False):
                    mode_at = now
                    anchor, anchor_at = pose, now
                # If a clear is already running, finish it before re-issuing a goal.
                if mode not in ('CANCEL', 'SKIP_CANCEL', 'BACKUP', 'BACKUP_STOP', 'CLEAR', 'REFRESH',
                                'POSITION_STOP', 'ALIGN_YAW', 'DEPART_YAW'):
                    mode = 'RESUME'
                    mode_at = now
                pause_at = None
            xy_error = math.hypot(point['x']-pose[0], point['y']-pose[1])
            if mode in ('SEND', 'MOVE') and xy_error <= point.get('xy_tolerance', self.p['xy_tolerance']):
                # Stop immediately on first XY entry, independent of action success,
                # yaw, remaining via-points or later localization position shifts.
                self.stop_pub.publish(Twist())
                self.spin_pub.publish(Twist())
                self.cancel_active_goal()
                self.state('POSITION_REACHED %s xy=%.3fm; position latched' % (point['name'], xy_error))
                mode, mode_at, last_cancel_notice = 'POSITION_STOP', now, -math.inf
            if mode == 'DEPART_YAW':
                tolerance = self.p.get('departure_yaw_tolerance', .15)
                bearing = math.atan2(point['y']-pose[1], point['x']-pose[0])
                if anchor is None:
                    anchor, anchor_at = pose, now
                if xy_error <= point.get('xy_tolerance', self.p['xy_tolerance']):
                    aligned, error = True, 0.
                    self.spin_pub.publish(Twist())
                else:
                    aligned, error = self.align_heading(bearing, pose[2], tolerance)
                if aligned:
                    rospy.loginfo('[Patrol] Departure heading aligned: error=%.3frad', error)
                    mode = 'SEND'
                else:
                    rospy.loginfo_throttle(3., '[Patrol] Departure %s yaw_error=%.3frad',
                                           point['name'], error)
                    # Reuse measured-motion stall detection: a blocked departure
                    # spin must back away, not remain in this phase indefinitely.
                    distance, yaw_change = motion_delta(anchor, pose)
                    if distance >= self.p['stall_distance'] or yaw_change >= self.p['stall_yaw']:
                        anchor, anchor_at = pose, now
                    stall_window = (self.p['near_goal_stall_time'] if xy_error < self.p['near_goal_distance']
                                    else self.p['stall_time'])
                    if is_stalled(anchor, pose, now-anchor_at, stall_window,
                                  self.p['stall_distance'], self.p['stall_yaw']):
                        self.stall_report(point, pose, anchor, now-anchor_at)
                        self.diagnostic('DEPARTURE_TURN_BLOCKED')
                        mode, retries = self.begin_recovery(point, retries, max_backups)
                        mode_at, last_cancel_notice = now, -math.inf
            elif mode == 'SEND':
                self.plan_topics.clear()
                self.planner_events.clear()  # Diagnostics must belong to this goal/attempt.
                self.client.send_goal(self.goal(point))
                self.active = True
                deadline = now+self.p['goal_timeout']
                anchor, anchor_at = pose, now
                mode = 'MOVE'
                self.state('NAVIGATING ' + point['name'])
            elif mode in ('CANCEL', 'RESUME', 'POSITION_STOP', 'SKIP_CANCEL'):
                self.stop_pub.publish(Twist())
                if self.client.get_state() in TERMINAL or not self.active:
                    self.active = False
                    if mode == 'CANCEL':
                        mode, mode_at = 'BACKUP', now
                        self.state('BACKING_UP %s attempt=%d/%d' % (point['name'], retries, max_backups))
                    elif mode == 'POSITION_STOP':
                        self.active = False
                        mode = 'ALIGN_YAW'
                        self.state('ALIGN_YAW ' + point['name'])
                    elif mode == 'SKIP_CANCEL':
                        rospy.logwarn('[Patrol] Skipping %s after %d backup attempts; no navigable route',
                                      point['name'], retries)
                        self.state('SKIPPED ' + point['name'])
                        return None
                    else:
                        mode = 'DEPART_YAW' if align_departure else 'SEND'
                        anchor = None
                        if align_departure:
                            self.state('ALIGN_DEPARTURE ' + point['name'])
                elif now-mode_at > self.p.get('cancel_wait_timeout', 15.):
                    self.diagnostic('CANCEL_TIMEOUT')
                    rospy.logerr('[Patrol] move_base still did not acknowledge cancellation after %.1fs; held stopped',
                                 now-mode_at)
                    return False
                elif now-mode_at > self.p['cancel_timeout'] and now-last_cancel_notice >= self.p['cancel_timeout']:
                    self.cancel_active_goal()
                    rospy.logwarn('[Patrol] CANCEL_PENDING %.1fs; planner may still be computing; holding zero', now-mode_at)
                    self.diagnostic('CANCEL_PENDING')
                    last_cancel_notice = now
            elif mode == 'ALIGN_YAW':
                # Navigation's time budget must not cut short heading alignment.
                # Finish on measured heading, still respecting normal pause/stop.
                aligned, error = self.align_heading(point.get('yaw', pose[2]), pose[2],
                                                     point.get('yaw_tolerance', self.p['yaw_tolerance']))
                if aligned:
                    self.stop_pub.publish(Twist())
                    rospy.loginfo('[Patrol] Yaw aligned: error=%.3frad; XY remains latched', error)
                    return True
                rospy.loginfo_throttle(3., '[Patrol] Aligning %s yaw_error=%.3frad', point['name'], error)
            elif mode == 'BACKUP':
                self.spin_pub.publish(Twist())
                command = Twist()
                if now-mode_at < self.p.get('backup_duration', 2.2):
                    command.linear.x = self.p.get('backup_speed', -.20)
                    # Mission has priority over navigation in the existing mux;
                    # driver slew, PWM limits and MCU watchdog remain unchanged.
                    self.stop_pub.publish(command)
                else:
                    self.stop_pub.publish(command)
                    mode, mode_at = 'BACKUP_STOP', now
            elif mode == 'BACKUP_STOP':
                self.stop_pub.publish(Twist())
                self.spin_pub.publish(Twist())
                # One existing mux/watchdog interval at zero before clearing.
                if now-mode_at >= .5:
                    result, mode, mode_at = self.clear_async(), 'CLEAR', now
            elif mode == 'CLEAR':
                self.stop_pub.publish(Twist())
                try:
                    ok, cleared_at, error = result.get_nowait()
                    if not ok:
                        rospy.logwarn('[Patrol] Clear unavailable: %s; retrying native planner', error)
                    mode, mode_at = 'REFRESH', now
                except queue.Empty:
                    if now-mode_at > self.p['recovery_timeout']:
                        rospy.logerr('[Patrol] Costmap clear service did not return')
                        return False
            elif mode == 'REFRESH':
                self.stop_pub.publish(Twist())
                # Stay cancelled until real scans and a new local grid have arrived.
                if min(self.scan_at, self.costmap_at) > cleared_at and now-cleared_at >= .5:
                    self.state('RECOVERED ' + point['name'])
                    mode = 'DEPART_YAW' if align_departure else 'SEND'
                    anchor = None
                    if align_departure:
                        self.state('ALIGN_DEPARTURE ' + point['name'])
                elif now-mode_at > self.p['recovery_timeout']:
                    rospy.logerr('[Patrol] No fresh scan/costmap after clearing')
                    return False
            else:
                action_state = self.client.get_state()
                if action_state == GoalStatus.SUCCEEDED:
                    self.active = False
                    rospy.logwarn('[Patrol] Action succeeded outside XY tolerance: xy=%.3fm', xy_error)
                distance, yaw_change = motion_delta(anchor, pose)
                moved = distance >= self.p['stall_distance'] or yaw_change >= self.p['stall_yaw']
                if moved:
                    anchor, anchor_at = pose, now
                stall_window = (self.p['near_goal_stall_time'] if xy_error < self.p['near_goal_distance']
                                else self.p['stall_time'])
                stalled = is_stalled(anchor, pose, now-anchor_at, stall_window,
                                     self.p['stall_distance'], self.p['stall_yaw'])
                no_local = self.plan_count('/move_base/TebLocalPlannerROS/local_plan', now) == 0
                if stalled and now-last_stall >= stall_window:
                    self.stall_report(point, pose, anchor, now-anchor_at)
                    last_stall = now
                # A nonempty TEB band is not evidence that the chassis can move.
                # Reuse measured XY/yaw stall detection even when that band keeps
                # updating but the outlet cannot execute it near an obstacle.
                retry = action_state in TERMINAL or stalled or now >= deadline
                if retry:
                    self.diagnostic('RECOVERY_TRIGGER no_local=%s stalled=%s' % (no_local, stalled))
                    mode, retries = self.begin_recovery(point, retries, max_backups)
                    mode_at, last_cancel_notice = now, -math.inf
                rospy.loginfo_throttle(5., '[Patrol] En route to %s...', point['name'])
            time.sleep(.05)
        self.cancel_active_goal()
        self.spin_pub.publish(Twist())
        self.stop_pub.publish(Twist())
        return False

    def run(self):
        self.wait_initial()
        skipped = []
        for cycle in range(int(self.p['cycles'])):
            for index, point in enumerate(self.data['waypoints'], 1):
                if not self.running():
                    return
                point.setdefault('name', 'point_%03d' % index)
                self.state('MOVING cycle=%d point=%s' % (cycle+1, point['name']))
                started_at = time.monotonic()
                reached = self.navigate(point, align_departure=True)
                if not self.running():
                    return
                if reached is None:
                    skipped.append(point['name'])
                    continue
                if not reached:
                    if not self.running():
                        return
                    raise RuntimeError('Failed to reach ' + point['name'])
                self.state('REACHED %s elapsed=%.3fs' % (point['name'], time.monotonic()-started_at))
                end = time.monotonic()+point.get('wait_s', self.p['wait_s'])
                while self.running() and time.monotonic() < end:
                    time.sleep(.05)
        self.done_pub.publish(True)
        if skipped:
            rospy.logwarn('[Patrol] Sequence finished with skipped waypoints: %s', ', '.join(skipped))
        self.state('COMPLETED')


def main():
    rospy.init_node('slicer_multi_goal_patrol')
    patrol = None
    try:
        patrol = Patrol()
        patrol.run()
        if patrol.running() and not patrol.p.get('auto_exit', True):
            rospy.spin()
        return 0
    except (rospy.ROSInterruptException, KeyboardInterrupt):
        return 0
    except Exception as exc:
        if patrol:
            patrol.state('FAILED: ' + str(exc))
        rospy.logerr('[Patrol] %s', exc)
        return 1
    finally:
        if patrol:
            patrol.shutdown()


if __name__ == '__main__':
    sys.exit(main())
