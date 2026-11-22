#!/usr/bin/env python3
"""One geographic base_link goal through the existing Slicer MoveBaseAction.

Latitude/longitude and origin files assert WGS84 ellipsoid coordinates. ENU
means metres East/North/Up. No datum conversion or physical arrival claim.
WGS84 sends can request shared preparation. Plan and capture do not move.
"""
import argparse
from collections import deque
import json
import math
from pathlib import Path
import sys
import threading
import time
import uuid

sys.path.insert(0, str(Path(__file__).resolve().parent))

from navigation_preparation import PreparationClient
from geographic_goal_core import (arrival_review, capture_sample, capture_target, finite, origin_record,
                                  quiet_odom_tail, registration_observation_problem,
                                  resolve_target, status_problem)


def number(value):
    result = float(value)
    if not math.isfinite(result):
        raise argparse.ArgumentTypeError('must be finite')
    return result


def positive(value):
    result = number(value)
    if result <= 0:
        raise argparse.ArgumentTypeError('must be greater than zero')
    return result


def arguments(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--action', default='/ugv/move_base')
    commands = parser.add_subparsers(dest='command', required=True)
    commands.add_parser('status', help='print one fresh manager status JSON')
    initialize = commands.add_parser('initialize', help='one bounded forward odom goal between stationary windows')
    initialize.add_argument('--distance', type=positive, default=2.5, help='finite metres forward; choose available field distance, no automatic retry')
    initialize.add_argument('--timeout', type=positive, default=120.)
    initialize.add_argument('--output', type=Path)
    capture = commands.add_parser('capture', help='save a stationary raw-RTK base_link target; no motion or initialization')
    capture.add_argument('--duration', type=positive, default=3., help='seconds of new stationary data, default 3')
    capture.add_argument('--output', type=Path, required=True, help='new WGS84 target JSON; never overwrites')
    for command in ('send', 'plan'):
        target = commands.add_parser(command, help='submit one Action goal' if command == 'send' else 'GetPlan only')
        target.add_argument('--latitude', type=number, help='WGS84 degrees')
        target.add_argument('--longitude', type=number, help='WGS84 degrees')
        target.add_argument('--altitude-ellipsoid', type=number, help='WGS84 ellipsoid metres, never MSL')
        target.add_argument('--east', type=number)
        target.add_argument('--north', type=number)
        target.add_argument('--up', type=number, default=0.)
        target.add_argument('--target-file', type=Path, help='WGS84 target JSON saved by capture; portable across initialization')
        origin = target.add_mutually_exclusive_group()
        origin.add_argument('--origin-file', type=Path, help='JSON datum WGS84, origin, optional axes ENU and units m')
        origin.add_argument('--session-enu', action='store_true', help='assert coordinates use current fixed task origin/ENU axes')
        target.add_argument('--yaw', type=number, help='radians CCW from input ENU East; omitted: approach bearing')
        target.add_argument('--timeout', type=positive, default=120.)
        target.add_argument('--output', type=Path, help='save result as a new JSON file')
    args = parser.parse_args(argv)
    if args.command in ('send', 'plan'):
        geodetic = any(x is not None for x in (args.latitude, args.longitude, args.altitude_ellipsoid))
        enu = args.east is not None or args.north is not None
        if sum((geodetic, enu, args.target_file is not None)) != 1:
            parser.error('provide one of WGS84 coordinates, ENU coordinates or --target-file')
        if args.target_file is not None:
            if args.origin_file is not None or args.session_enu:
                parser.error('a WGS84 target file does not take an ENU origin selector')
        elif geodetic:
            if any(x is None for x in (args.latitude, args.longitude, args.altitude_ellipsoid)):
                parser.error('WGS84 input requires latitude, longitude and ellipsoid altitude')
            if not -90. <= args.latitude <= 90. or not -180. <= args.longitude <= 180.:
                parser.error('WGS84 latitude/longitude outside bounds')
            if args.origin_file is not None or args.session_enu:
                parser.error('origin selection applies to ENU input only')
        elif args.east is None or args.north is None or not (args.origin_file or args.session_enu):
            parser.error('ENU input requires east, north and --origin-file or --session-enu')
    return args


class GeographicClient:
    def __init__(self, args):
        import actionlib
        from actionlib_msgs.msg import GoalStatusArray
        from geometry_msgs.msg import PoseStamped
        from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
        from nav_msgs.msg import Odometry
        from nav_msgs.srv import GetPlan, GetPlanRequest
        import rospy
        from std_msgs.msg import String
        from std_srvs.srv import Trigger
        import tf2_ros
        self.args, self.ros = args, rospy
        self.String, self.Trigger = String, Trigger
        self.PoseStamped, self.MoveBaseGoal = PoseStamped, MoveBaseGoal
        self.GetPlan, self.GetPlanRequest = GetPlan, GetPlanRequest
        self.lock, self.status, self.received = threading.Lock(), None, None
        self.goal_record = None
        self.odom = deque(maxlen=500)
        self.fixes, self.qualities, self.global_poses = {}, {}, deque(maxlen=500)
        self.owned = False
        self.client = actionlib.SimpleActionClient(args.action, MoveBaseAction)
        self.status_age = rospy.get_param('/geographic/status_max_age_s', 1.)
        self.fix_age = rospy.get_param('/geographic/max_fix_age_s', 2.5)
        self.pose_gap = rospy.get_param('/geographic/max_odom_gap_s', .5)
        self.stop_window = rospy.get_param('/geographic/stop_window_s', 1.)
        self.tolerance = rospy.get_param('/geographic/arrival_tolerance_m', .35)
        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer)
        self.status_sub = rospy.Subscriber('/geographic/status', String, self._status, queue_size=10)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self._odom, queue_size=100)
        self.goal_pub = rospy.Publisher('/geographic/goal', String, queue_size=1, latch=True)
        self.result_pub = rospy.Publisher('/geographic/goal_result', String, queue_size=1, latch=True)
        self.preparation = PreparationClient(rospy, String, args.action)
        self.capture_subs = []
        self.action_sample, self.mux_sample = None, None
        if args.command in ('send', 'initialize'):
            self.action_sub = rospy.Subscriber(args.action.rstrip('/') + '/status', GoalStatusArray,
                                               self._action_status, queue_size=1)
            self.mux_sub = rospy.Subscriber('/cup_nav/manual_state', String, self._mux_status, queue_size=1)
        if args.command == 'capture':
            from sensor_msgs.msg import NavSatFix
            from diagnostic_msgs.msg import DiagnosticArray
            self.capture_subs = [
                rospy.Subscriber('/gnss/fix', NavSatFix, self._fix, queue_size=100),
                rospy.Subscriber('/gnss/fix_quality', DiagnosticArray, self._quality, queue_size=100),
                rospy.Subscriber('/global/odom', Odometry, self._global, queue_size=100)]

    def _action_status(self, message):
        expected = self.args.action.rstrip('/')
        if message._connection_header.get('callerid') == expected:
            active = any(status.status in (0, 1, 6, 7) for status in message.status_list)
            self.action_sample = time.monotonic(), message.header.stamp.to_sec(), active

    def _mux_status(self, message):
        header = message._connection_header
        if header.get('callerid') != '/slicer_velocity_mux' or header.get('latching', '0') != '0':
            return
        try:
            state = json.loads(message.data)
            flags = [state[name] for name in ('manual_hold', 'teleop_fresh', 'emergency_stop_latched')]
            if all(type(flag) is bool for flag in flags):
                self.mux_sample = time.monotonic(), any(flags) or state.get('selected_source') == 'mission'
        except (KeyError, TypeError, ValueError):
            pass

    def check_action_idle(self):
        if getattr(self.args, 'command', None) not in ('send', 'initialize'):
            return
        sample = self.action_sample
        if (not sample or time.monotonic() - sample[0] > .6
                or not -.1 <= self.ros.Time.now().to_sec() - sample[1] <= .6):
            raise ValueError('Action status unavailable or stale; no goal sent')
        if sample[2] or self.preparation_active():
            raise ValueError('another task owns navigation; no goal sent')
        mux = self.mux_sample
        if not mux or time.monotonic() - mux[0] > .6 or mux[1]:
            raise ValueError('Mux not available for navigation; no goal sent')

    def _status(self, message):
        try:
            value = json.loads(message.data)
            if not isinstance(value, dict):
                value = None
        except (ValueError, TypeError):
            value = None
        with self.lock:
            self.status, self.received = value, time.monotonic()

    def _odom(self, message):
        velocity = message.twist.twist
        position = message.pose.pose.position
        row = dict(received=time.monotonic(), stamp=message.header.stamp.to_sec(),
                   speed=math.hypot(velocity.linear.x, velocity.linear.y), angular_speed=velocity.angular.z,
                   x=position.x, y=position.y)
        with self.lock:
            self.odom.append(row)

    def observation(self):
        with self.lock:
            return self.status, self.received, list(self.odom)

    def _fix(self, message):
        row = dict(stamp=message.header.stamp.to_sec(), stamp_ns=message.header.stamp.to_nsec(),
                   received=time.monotonic(), frame=message.header.frame_id, status=message.status.status,
                   latitude=message.latitude, longitude=message.longitude, altitude=message.altitude)
        with self.lock:
            self.fixes[row['stamp_ns']] = row
            if len(self.fixes) > 500:
                del self.fixes[min(self.fixes)]

    def _quality(self, message):
        for status in message.status:
            if status.name == 'cup_gnss/receiver':
                row = {item.key: item.value for item in status.values}
                row['stamp_ns'] = message.header.stamp.to_nsec()
                with self.lock:
                    self.qualities[row['stamp_ns']] = row
                    if len(self.qualities) > 500:
                        del self.qualities[min(self.qualities)]

    def _global(self, message):
        if message.header.frame_id != 'map' or message.child_frame_id != 'base_link':
            return
        p, q = message.pose.pose.position, message.pose.pose.orientation
        if (not all(finite(v) for v in (p.x, p.y, q.x, q.y, q.z, q.w)) or
                not .99 <= sum(v*v for v in (q.x, q.y, q.z, q.w)) <= 1.01):
            return
        row = dict(stamp=message.header.stamp.to_sec(), x=p.x, y=p.y,
                   yaw=math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))
        with self.lock:
            if not self.global_poses or row['stamp'] > self.global_poses[-1]['stamp']:
                self.global_poses.append(row)

    def capture_inputs(self):
        with self.lock:
            return list(self.fixes.values()), dict(self.qualities), list(self.global_poses)

    def fresh_status(self, registered=True):
        deadline = time.monotonic()+5.
        while not self.ros.is_shutdown() and time.monotonic() < deadline:
            status, received, _ = self.observation()
            if status is not None and received is not None:
                if registered:
                    problem = status_problem(status, received, time.monotonic(), self.ros.Time.now().to_sec(),
                                             max_age=self.status_age)
                    if problem:
                        raise ValueError(problem)
                elif not (time.monotonic()-received <= self.status_age and
                          -.1 <= self.ros.Time.now().to_sec()-status.get('status_stamp', -1e9) <= self.status_age):
                    raise ValueError('geographic status stale')
                return status
            time.sleep(.05)
        raise ValueError('geographic status unavailable')

    def current_pose(self, frame):
        transform = self.buffer.lookup_transform(frame, 'base_link', self.ros.Time(0), self.ros.Duration(3.))
        if abs((self.ros.Time.now()-transform.header.stamp).to_sec()) > .5:
            raise ValueError('base TF is stale')
        t, q = transform.transform.translation, transform.transform.rotation
        yaw = math.atan2(2.*(q.w*q.z+q.x*q.y), 1.-2.*(q.y*q.y+q.z*q.z))
        return t.x, t.y, yaw

    def pose(self, xyz, yaw, frame='map'):
        pose = self.PoseStamped()
        pose.header.frame_id, pose.header.stamp = frame, self.ros.Time.now()
        pose.pose.position.x, pose.pose.position.y = xyz[:2]
        # move_base is planar: retain geodetic Up in the record, use z=0 for navigation.
        pose.pose.orientation.z, pose.pose.orientation.w = math.sin(yaw/2.), math.cos(yaw/2.)
        return pose

    def frame_check(self, expected):
        key = self.args.action.rstrip('/')+'/global_costmap/global_frame'
        actual = self.ros.get_param(key, None)
        if actual != expected:
            raise ValueError('{} must be {}; actual={!r}'.format(key, expected, actual))

    def publish(self, publisher, record):
        publisher.publish(self.String(data=json.dumps(record, ensure_ascii=False, allow_nan=False)))

    def trigger(self, name):
        path = '/geographic/'+name
        self.ros.wait_for_service(path, timeout=5.)
        result = self.ros.ServiceProxy(path, self.Trigger)()
        try:
            payload = json.loads(result.message)
        except (ValueError, TypeError):
            payload = {'reason': result.message}
        return dict(success=result.success, details=payload)

    def initializing_problem(self, generation):
        status, received, _ = self.observation()
        if not isinstance(status, dict) or received is None:
            return 'geographic initialization status unavailable'
        if (time.monotonic()-received > self.status_age or
                not -.1 <= self.ros.Time.now().to_sec()-status.get('status_stamp', -1e9) <= self.status_age):
            return 'geographic initialization status stale'
        if status.get('generation') != generation or status.get('state') != 'collecting':
            return 'geographic initialization invalidated: '+str(status.get('reason', ''))
        return None

    def run_action(self, pose, generation, initializing=False):
        if not self.client.wait_for_server(self.ros.Duration(10.)):
            raise ValueError('Action server unavailable: '+self.args.action)
        problem = self.initializing_problem(generation) if initializing else status_problem(
            *self.observation()[:2], time.monotonic(), self.ros.Time.now().to_sec(),
            generation=generation, max_age=self.status_age)
        if problem:
            raise ValueError(problem)
        self.check_action_idle()
        self.client.send_goal(self.MoveBaseGoal(target_pose=pose))
        self.owned = True
        deadline, reason = time.monotonic()+self.args.timeout, ''
        try:
            while not self.client.wait_for_result(self.ros.Duration(.1)):
                status, received, _ = self.observation()
                reason = self.initializing_problem(generation) if initializing else status_problem(
                    status, received, time.monotonic(), self.ros.Time.now().to_sec(),
                    generation=generation, max_age=self.status_age)
                if not reason and (time.monotonic() >= deadline or self.ros.is_shutdown()):
                    reason = 'goal timeout' if not self.ros.is_shutdown() else 'ROS shutdown'
                if reason:
                    self.client.cancel_goal()
                    self.client.wait_for_result(self.ros.Duration(3.))
                    break
        except KeyboardInterrupt:
            reason = 'operator interrupted this client; owned goal cancellation requested'
            self.client.cancel_goal()
            self.client.wait_for_result(self.ros.Duration(3.))
        except BaseException:
            self.client.cancel_goal()
            self.client.wait_for_result(self.ros.Duration(3.))
            raise
        finally:
            self.owned = False
        return dict(state=self.client.get_state(), text=self.client.get_goal_status_text(),
                    reason=reason or '', ended_stamp=self.ros.Time.now().to_sec())

    def quiet(self, since, generation=None, initializing=False):
        window = max(2., self.stop_window) if initializing else self.stop_window
        deadline, result = time.monotonic()+max(4., window+4.), None
        while not self.ros.is_shutdown() and time.monotonic() < deadline:
            _, _, rows = self.observation()
            result = quiet_odom_tail([row for row in rows if row['received'] >= since], time.monotonic(),
                                     self.ros.Time.now().to_sec(), window=window)
            if result['confirmed']:
                return result
            if initializing:
                problem = self.initializing_problem(generation)
                if problem:
                    return dict(result, confirmed=False, reason=problem)
            time.sleep(.05)
        return result or {'confirmed': False, 'reason': 'ROS shutdown before quiet odometry'}

    def prepare_available(self):
        return bool(self.preparation.observation())

    def preparation_active(self):
        return self.preparation.observation().get('busy') is True

    def prepare_start(self, task_id=None, target=None):
        self.preparation.start(task_id, target)

    def prepare_poll(self):
        return self.preparation.poll()

    def prepare_cancel(self):
        self.preparation.cancel()

    def prepare_wait(self, task_id, target):
        self.prepare_start(task_id, target)
        try:
            while not self.ros.is_shutdown():
                phase, reason = self.prepare_poll()
                if phase == 'READY':
                    # Status and Action results arrive on separate ROS topics.
                    deadline = time.monotonic() + .5
                    while self.preparation_active() and time.monotonic() < deadline:
                        time.sleep(.02)
                    return
                if phase in ('FAILED', 'STOPPED'):
                    raise ValueError(reason)
                time.sleep(.05)
            raise ValueError('ROS shut down before geographic preparation completed')
        except KeyboardInterrupt:
            self.prepare_cancel()
            deadline = time.monotonic() + 10.
            while time.monotonic() < deadline and not self.ros.is_shutdown():
                phase, _ = self.prepare_poll()
                if phase in ('READY', 'FAILED', 'STOPPED'):
                    break
                time.sleep(.05)
            raise
        finally:
            self.prepare_cancel()

    def initialize(self):
        if self.preparation_active():
            raise ValueError('a WGS84 task is using shared navigation preparation')
        self.frame_check('odom')
        self.fresh_status(registered=False)
        began = self.trigger('begin_registration')
        if not began['success']:
            return dict(command='initialize', begin=began, reason='begin_registration rejected'), 1
        try:
            result, code = self._initialize(began)
        except BaseException:
            self._abort_manual_registration()
            raise
        if code:
            self._abort_manual_registration()
        return result, code

    def _abort_manual_registration(self):
        try:
            self.trigger('abort_registration')
        except (self.ros.ROSException, self.ros.ServiceException) as error:
            self.ros.logwarn('manual registration cleanup failed: %s', error)

    def _initialize(self, began):
        deadline = time.monotonic()+3.
        while time.monotonic() < deadline:
            status, _, _ = self.observation()
            if status and status.get('state') == 'collecting':
                break
            time.sleep(.05)
        if not status or status.get('state') != 'collecting':
            raise ValueError('begin_registration did not enter collecting state')
        generation = status['generation']
        initial_stop = self.quiet(time.monotonic(), generation, initializing=True)
        if not initial_stop['confirmed']:
            return dict(command='initialize', begin=began, initial_stop=initial_stop,
                        reason='stationary start window unavailable; no goal sent'), 1
        status = self.fresh_status(registered=False)
        problem = self.initializing_problem(generation) or registration_observation_problem(
            status, self.ros.Time.now().to_sec(), self.fix_age)
        if problem:
            return dict(command='initialize', begin=began, initial_stop=initial_stop, reason=problem), 1
        x, y, yaw = self.current_pose('odom')
        xyz = [x+self.args.distance*math.cos(yaw), y+self.args.distance*math.sin(yaw), 0.]
        record = dict(id=str(uuid.uuid4()), command='initialize', frame='odom', map_xyz=None,
                      odom_xyz=xyz, yaw=yaw, distance_m=self.args.distance, generation=generation,
                      physical_point='base_link', stamp=self.ros.Time.now().to_sec())
        self.goal_record = record
        self.publish(self.goal_pub, record)
        action = self.run_action(self.pose(xyz, yaw, 'odom'), generation, initializing=True)
        stopped = self.quiet(time.monotonic(), generation, initializing=True)
        # Finish is an estimation service, not permission to move or automatic retry.
        finished = self.trigger('finish_registration') if stopped['confirmed'] else None
        result = dict(command='initialize', target=record, begin=began, move_base=action,
                      stop=stopped, registration=finished,
                      reason=action['reason'] or ('' if finished and finished['success'] else 'registration not completed'))
        return result, 0 if action['state'] == 3 and not action['reason'] and finished and finished['success'] else 1

    def _wgs84_intent(self):
        if self.args.target_file is not None:
            from geographic_goal_core import load_wgs84_target
            return load_wgs84_target(self.args.target_file)
        if self.args.latitude is not None:
            return dict(latitude=self.args.latitude, longitude=self.args.longitude,
                        altitude_ellipsoid_m=self.args.altitude_ellipsoid)
        return None

    def target(self):
        goal_id, intent = str(uuid.uuid4()), None
        if self.args.command == 'send':
            intent = self._wgs84_intent()
            if intent:
                self.goal_record = dict(id=goal_id, original=intent, datum='WGS84',
                                        frame='wgs84', stamp=self.ros.Time.now().to_sec())
                self.publish(self.goal_pub, self.goal_record)
                # Wait briefly for the latched owner announcement in a fresh CLI process.
                deadline = time.monotonic() + 1.
                while not self.prepare_available() and time.monotonic() < deadline:
                    time.sleep(.05)
                if self.prepare_available():
                    self.prepare_wait(goal_id, intent)
                else:
                    try:
                        self.frame_check('map')
                    except ValueError as error:
                        raise ValueError('start slicer_navigation_runtime.launch for automatic WGS84 preparation') from error
            elif self.preparation_active():
                raise ValueError('navigation preparation is active')
        elif self.preparation_active():
            raise ValueError('preview unavailable while navigation preparation is active')
        self.frame_check('map')
        status = self.fresh_status()
        target = resolve_target(self.args, status, self.current_pose('map'), file_intent=intent)
        target.update(id=goal_id, generation=status['generation'], stamp=self.ros.Time.now().to_sec())
        self.goal_record = target
        pose = self.pose(target['map_xyz'], target['yaw'])
        if self.args.command == 'send':
            self.publish(self.goal_pub, target)
        if self.args.command == 'plan':
            start = self.current_pose('map')
            service = self.args.action.rstrip('/')+'/make_plan'
            self.ros.wait_for_service(service, timeout=5.)
            before = time.monotonic()
            plan = self.ros.ServiceProxy(service, self.GetPlan)(self.GetPlanRequest(
                start=self.pose([start[0], start[1], 0.], start[2]), goal=pose, tolerance=0.))
            result = dict(command='plan', target=target, poses=len(plan.plan.poses),
                          elapsed_s=time.monotonic()-before, action_goal_sent=False)
            return result, 0 if plan.plan.poses else 1
        action = self.run_action(pose, target['generation'])
        since = time.monotonic()
        self.quiet(since)
        result = self.arrival_after_stop(target, action, since)
        result.update(command='send', move_base=action)
        return result, 0 if result['geographic']['confirmed'] and not action['reason'] else 1

    def arrival_after_stop(self, target, action, since):
        """Bounded wait for an RTK epoch inside the continuously observed quiet window."""
        deadline = time.monotonic()+self.fix_age
        while True:
            status, received, rows = self.observation()
            now, ros_now = time.monotonic(), self.ros.Time.now().to_sec()
            # Recheck even on the first iteration: motion/staleness observed
            # since quiet() returned must invalidate that earlier stopped tail.
            stopped = quiet_odom_tail([row for row in rows if row['received'] >= since],
                                      now, ros_now, window=self.stop_window)
            result = arrival_review(target, status, received, now, ros_now, target['generation'],
                                    action['state'], stopped, self.tolerance, self.fix_age, self.status_age)
            if (result['geographic']['qualified'] or not stopped.get('confirmed') or action['state'] != 3
                    or now >= deadline or self.ros.is_shutdown()):
                if not result['geographic']['qualified']:
                    result['geographic']['wait_limit_s'] = self.fix_age
                return result
            time.sleep(.05)

    def capture(self):
        """Sample only: no Trigger, Action, Mux or localization-state writes."""
        status = self.fresh_status()
        origin, generation = origin_record(status), status['generation']
        lever = status.get('antenna_in_base_link_xy_m')
        if not isinstance(lever, (list, tuple)) or len(lever) != 2 or not all(finite(x) for x in lever):
            raise ValueError('capture: configured horizontal antenna lever unavailable')
        since, start_stamp = time.monotonic(), self.ros.Time.now().to_sec()
        window = max(self.args.duration, self.stop_window)
        deadline = since+window+self.fix_age+1.
        accepted, problem = {}, 'no new qualified GNSS/quality/pose samples'
        while not self.ros.is_shutdown() and time.monotonic() < deadline:
            status, received, rows = self.observation()
            now, ros_now = time.monotonic(), self.ros.Time.now().to_sec()
            invalid = status_problem(status, received, now, ros_now, generation, self.status_age)
            if invalid:
                raise ValueError('capture: '+invalid)
            fixes, qualities, poses = self.capture_inputs()
            for fix in fixes:
                key = fix['stamp_ns']
                if (key in accepted or key not in qualities or fix['received'] < since
                        or fix['stamp'] < start_stamp):
                    continue
                try:
                    accepted[key] = capture_sample(fix, qualities[key], poses, lever, origin,
                                                   ros_now, self.fix_age, self.pose_gap)
                except ValueError as error:
                    problem = str(error)  # missing pose may acquire its bracket on the next callback
            stopped = quiet_odom_tail([row for row in rows if row['received'] >= since], now, ros_now, window=window)
            if stopped['confirmed'] and status.get('state') == 'ready':
                try:
                    result = capture_target(list(accepted.values()), origin, stopped, self.args.duration)
                    result['capture'].update(antenna_in_base_link_xy_m=list(lever), source_generation=generation)
                    return result, 0
                except ValueError as error:
                    problem = str(error)
            elif not stopped['confirmed']:
                problem = stopped['reason']
            else:
                problem = 'geographic localization not ready: '+str(status.get('reason', ''))
            time.sleep(.05)
        raise ValueError('capture unavailable within bounded sample window: '+problem)


def main(argv=None):
    args = arguments(argv)
    try:
        import rospy
        import tf2_ros
    except ImportError as error:
        print('Run in the built ROS1 Noetic workspace: '+str(error), file=sys.stderr)
        return 2
    rospy.init_node('slicer_geographic_client', anonymous=True, disable_signals=True)
    client = None
    try:
        client = GeographicClient(args)
        if args.command == 'status':
            result, code = client.fresh_status(registered=False), 0
        else:
            if args.command == 'capture':
                result, code = client.capture()
            else:
                result, code = client.initialize() if args.command == 'initialize' else client.target()
            client.publish(client.result_pub, result)
            # Give connected recorders time to receive the latched final result.
            time.sleep(.2)
        output = json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False)
        if getattr(args, 'output', None) is not None:
            with args.output.open('x', encoding='utf-8') as stream:
                stream.write(output+'\n')
        print(output)
        return code
    except KeyboardInterrupt:
        return 130
    except (OSError, ValueError, TypeError, rospy.ROSException, rospy.ServiceException,
            tf2_ros.TransformException) as error:
        result = {'command': args.command, 'reason': str(error)}
        if client is not None and args.command != 'status':
            result['target'] = client.goal_record
            client.publish(client.result_pub, result)
            time.sleep(.2)
        output = json.dumps(result, ensure_ascii=False, indent=2)
        if args.command != 'capture' and getattr(args, 'output', None) is not None and not args.output.exists():
            try:
                with args.output.open('x', encoding='utf-8') as stream:
                    stream.write(output+'\n')
            except OSError:
                pass
        print(output, file=sys.stderr)
        return 1
    finally:
        if client is not None and client.owned:
            client.client.cancel_goal()
            client.client.wait_for_result(rospy.Duration(3.))
        rospy.signal_shutdown('geographic client finished')


if __name__ == '__main__':
    sys.exit(main())
