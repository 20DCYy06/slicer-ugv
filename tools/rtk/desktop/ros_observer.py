"""Observe the existing RTK stack and dispatch only explicit desktop goals."""
from copy import deepcopy
import json
import math
from threading import Lock
import time
import uuid

import rospy
import tf2_geometry_msgs
import tf2_ros
from actionlib_msgs.msg import GoalStatusArray
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PointStamped, PoseStamped
from move_base_msgs.msg import MoveBaseActionGoal
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan, NavSatFix
from std_msgs.msg import Bool, String
from slicer_air_ground_msgs.msg import Command

from geographic_core import enu_to_geodetic
from geographic_state import qualified_fix
from dashboard_model import (ACTIVE_ACTION_STATES, FIX_POSITION_AGE, NAV_STATUS_AGE, coordinates,
                             manual_goal_block, task_destination)


ACTION = '/ugv/move_base'
# The wrapper gives each desktop its own ROS namespace.
GOAL_CLICK = 'manual_goal'
WAYPOINT_CLICK = 'waypoint'
GOAL_STATUS_AGE = .5
GOAL_WAIT = .65
FIX_BUFFER_SIZE = 64
WAYPOINT_PROMPT = '等待地理配准'


def finite(*values):
    return all(isinstance(value, (float, int)) and not isinstance(value, bool)
               and math.isfinite(value) for value in values)


def recent_stamp(stamp, limit):
    return finite(stamp) and stamp > 0 and -.15 <= rospy.Time.now().to_sec() - stamp <= limit


def json_sample(message, timestamp):
    try:
        value = json.loads(message.data)
    except (ValueError, TypeError):
        return None
    return value if isinstance(value, dict) and recent_stamp(value.get(timestamp), 2.5) else None


class Observer:
    def __init__(self, board):
        self.board = board
        self.tf = tf2_ros.Buffer(cache_time=rospy.Duration(5.))
        self.listener = tf2_ros.TransformListener(self.tf)
        self.lock = Lock()
        self.goal_id = None
        self.goal_sent = 0.
        self.desktop_goals = {}
        self.manual_target = None
        self.manual_feedback = None
        self.pending_goal = None
        self.calibration_active = False
        self.action_connection = None
        self.action_goals = {}
        self.fixed_fixes = {}
        self.fixed_qualities = {}
        self.fixed_stamp_ns = 0
        self.last_quality_stamp_ns = 0
        self.fixed_session = None
        self.waypoint_selected = None
        self.waypoint_status = f'Waypoint：{WAYPOINT_PROMPT}'
        self.waypoint_coords = ''
        self.goal_pub = rospy.Publisher(ACTION + '/goal', MoveBaseActionGoal, queue_size=1)
        self.subscribers = []
        for topic, kind, callback in (
            ('/gnss/diagnostics', DiagnosticArray, self.diagnostics),
            ('/gnss/fix', NavSatFix, self.gnss_fix),
            ('/gnss/fix_quality', DiagnosticArray, self.fix_quality),
            ('/geographic/status', String, self.geographic),
            ('/geographic/navigation_runtime', String, self.navigation_runtime),
            ('/scan', LaserScan, lambda m: self.sensor('lidar', m)),
            ('/lio/odom', Odometry, lambda m: self.sensor('lio', m)),
            ('/odom', Odometry, self.local_odom),
            ('/wheel/odom', Odometry, self.wheel_odom),
            ('/global/odom', Odometry, self.global_odom),
            ('/slicer_c30d_driver/ready', Bool, lambda m: self.board.record('driver', m.data)),
            ('/cup_nav/manual_state', String, self.mux),
            (ACTION + '/global_costmap/costmap', OccupancyGrid, self.costmap),
            (ACTION + '/status', GoalStatusArray, self.action_status),
            (ACTION + '/goal', MoveBaseActionGoal, self.action_goal),
            ('/air_ground/state', String, lambda m: self.task('v1', m, 'updated_stamp')),
            ('/air_ground/v2/state', String, lambda m: self.task('v2', m, 'updated_at')),
            ('/air_ground/v2/uav_state', String, self.aircraft_v2),
            ('/uav/test', String, self.aircraft_v1),
            ('/air_ground/command', Command, self.aircraft_v1_command),
            (GOAL_CLICK, PoseStamped, self.manual_goal),
            (WAYPOINT_CLICK, PoseStamped, self.waypoint),
        ):
            self.subscribers.append(rospy.Subscriber(topic, kind, callback, queue_size=1))
        self.frame_timer = rospy.Timer(rospy.Duration(.5), self.planner_frame)

    def quality(self, key, message):
        if not recent_stamp(message.header.stamp.to_sec(), 3.):
            return
        values = next(({pair.key: pair.value for pair in item.values}
                       for item in message.status if item.name == 'cup_gnss/receiver'), None)
        if values is not None:
            self.board.record(key, values)
        return values

    def diagnostics(self, message):
        self.quality('gnss', message)

    def gnss_fix(self, message):
        self._gnss_observation(message, dict(frame=message.header.frame_id, status=message.status.status,
                                           latitude=message.latitude, longitude=message.longitude,
                                           altitude=message.altitude), quality=False)

    def fix_quality(self, message):
        values = self.quality('quality', message)
        if values is not None:
            self._gnss_observation(message, values, quality=True)

    def _gnss_observation(self, message, values, *, quality):
        timestamp = message.header.stamp
        stamp_ns, stamp = timestamp.to_nsec(), timestamp.to_sec()
        ros_now = rospy.Time.now().to_sec()
        received = time.monotonic()
        if stamp_ns <= 0 or not 0 <= ros_now-stamp <= FIX_POSITION_AGE:
            return
        header = getattr(message, '_connection_header', None) or {}
        if not header.get('callerid') or header.get('latching', '0') != '0':
            return
        sample = dict(values, stamp_ns=stamp_ns, stamp=stamp, callerid=header['callerid'])
        with self.lock:
            if stamp_ns <= self.fixed_stamp_ns:
                return
            if quality and not self._store_fix_quality(sample):
                return
            if not quality:
                self._store_fix_sample(self.fixed_fixes, stamp_ns, sample)
            if stamp_ns not in self.fixed_fixes or stamp_ns not in self.fixed_qualities:
                return
            fix = self.fixed_fixes.pop(stamp_ns)
            diag = self.fixed_qualities.pop(stamp_ns)
            self.fixed_stamp_ns = stamp_ns
            point = None
            if fix['callerid'] == diag['callerid'] and not qualified_fix(fix, diag, ros_now, FIX_POSITION_AGE):
                point = {key: fix[key] for key in ('latitude', 'longitude', 'altitude')}
            self.board.record('fixed_position', point, received-(ros_now-stamp))

    @staticmethod
    def _store_fix_sample(samples, stamp, value):
        samples[stamp] = value
        if len(samples) > FIX_BUFFER_SIZE:
            del samples[min(samples)]

    def _store_fix_quality(self, sample):
        stamp_ns = sample['stamp_ns']
        session = sample.get('session')
        if stamp_ns > self.last_quality_stamp_ns:
            self.last_quality_stamp_ns = stamp_ns
            if session != self.fixed_session:
                # Prevent a late position from the previous session restoring the readout.
                self.fixed_session = session
                self.fixed_qualities.clear()
                self.fixed_stamp_ns = stamp_ns - 1
                self.board.record('fixed_position', None)
        if session != self.fixed_session:
            return False
        usable = (session and str(sample.get('gga_quality')) == '4'
                  and str(sample.get('fresh_rtk_usable')).lower() == 'true'
                  and str(sample.get('fix_valid')).lower() == 'true')
        if not usable:
            self.fixed_stamp_ns = stamp_ns
            self.board.record('fixed_position', None)
            return False
        self._store_fix_sample(self.fixed_qualities, stamp_ns, sample)
        return True

    def geographic(self, message):
        value = json_sample(message, 'status_stamp')
        if value is not None:
            self.board.record('geographic', value)

    def navigation_runtime(self, message):
        value = json_sample(message, 'stamp')
        if value is not None:
            self.board.record('runtime', value)

    def sensor(self, key, message):
        if recent_stamp(message.header.stamp.to_sec(), 2.5):
            self.board.record(key, True)

    def wheel_odom(self, message):
        stamp = message.header.stamp.to_sec()
        age = rospy.Time.now().to_sec() - stamp
        if stamp <= 0 or not -.15 <= age <= 1.:
            return
        speed = message.twist.twist
        self.board.record('wheel_odom', True)
        header = getattr(message, '_connection_header', None) or {}
        if header.get('callerid') == '/slicer_c30d_driver':
            self.board.record('wheel_still', finite(speed.linear.x, speed.angular.z)
                              and abs(speed.linear.x) <= .02 and abs(speed.angular.z) <= .02,
                              time.monotonic() - max(0., age))

    def local_odom(self, message):
        stamp = message.header.stamp.to_sec()
        if not recent_stamp(stamp, 1.):
            return
        p, speed = message.pose.pose.position, message.twist.twist
        if message.header.frame_id != 'odom' or not finite(p.x, p.y, speed.linear.x, speed.angular.z):
            return
        self.board.record('odom', (p.x, p.y))
        self.board.record('odom_still', abs(speed.linear.x) <= .02 and abs(speed.angular.z) <= .02)

    def global_odom(self, message):
        p = message.pose.pose.position
        if (message.header.frame_id == 'map' and recent_stamp(message.header.stamp.to_sec(), 1.)
                and finite(p.x, p.y)):
            self.board.record('map_odom', (p.x, p.y))

    def mux(self, message):
        try:
            value = json.loads(message.data)
        except (TypeError, ValueError):
            return
        if isinstance(value, dict) and value.get('schema_version') == 1:
            self.board.record('mux', value)

    def costmap(self, message):
        if message.header.frame_id in ('odom', 'map') and recent_stamp(message.header.stamp.to_sec(), 2.5):
            self.board.record('frame', message.header.frame_id)

    def planner_frame(self, _event):
        # Full grids may be published once, followed by incremental updates only.
        with self.lock:
            connection = self.action_connection
        if self.board.recent('action', time.monotonic(), NAV_STATUS_AGE) is None:
            return
        frame = rospy.get_param(ACTION + '/global_costmap/global_frame', None)
        with self.lock:
            if connection is self.action_connection and frame in ('odom', 'map'):
                self.board.record('configured_frame', frame)

    def action_status(self, message):
        header = getattr(message, '_connection_header', None)
        if not header or header.get('callerid') != ACTION:
            return
        stamp = message.header.stamp.to_sec()
        age = rospy.Time.now().to_sec() - stamp
        if stamp <= 0 or not -.15 <= age <= NAV_STATUS_AGE:
            return
        items = [(item.goal_id.id, item.status) for item in message.status_list]
        reported = dict(items)
        now = time.monotonic()
        with self.lock:
            reconnected = header is not self.action_connection
            if reconnected:
                # rospy shares the header dictionary among messages on one TCPROS connection.
                self.action_connection = header
                self.board.record('configured_frame', None, now)
                self._cancel_pending_goal_locked('导航已重新连接，请重新点选')
            self.board.record('action', items, now - max(0., age))
            self.desktop_goals = {
                goal: sent for goal, sent in self.desktop_goals.items()
                if reported.get(goal) in ACTIVE_ACTION_STATES
                or (not reconnected and goal not in reported and now - sent <= 2.)
            }
            if self.goal_id is not None:
                if self.goal_id not in self.desktop_goals:
                    self.goal_id = None
            if self.pending_goal is not None and age <= GOAL_STATUS_AGE:
                pending, deadline = self.pending_goal
                self.pending_goal = None
                if now <= deadline:
                    self._dispatch_goal_locked(pending, now)
                else:
                    self._goal_blocked_locked('未收到新的 move_base 状态，请重新点选', now)

    def action_goal(self, message):
        header = getattr(message, '_connection_header', None) or {}
        if header.get('callerid', '').startswith('/slicer_navigation_runtime'):
            return
        pose = message.goal.target_pose
        p = pose.pose.position
        if pose.header.frame_id in ('odom', 'map') and finite(p.x, p.y, p.z) and message.goal_id.id:
            with self.lock:
                self.action_goals[message.goal_id.id] = pose, time.monotonic()
                if len(self.action_goals) > 32:
                    self.action_goals.pop(next(iter(self.action_goals)))

    def task(self, key, message, timestamp):
        value = json_sample(message, timestamp)
        if value is not None:
            self.board.record(key, value)

    def aircraft_v1(self, message):
        header = getattr(message, '_connection_header', None) or {}
        if header.get('latching') == '0' and message.data.strip():
            self.board.record('aircraft_v1', header.get('callerid'))

    def aircraft_v1_command(self, message):
        header = getattr(message, '_connection_header', None) or {}
        local = self.board.recent('v1', time.monotonic())
        if (header.get('latching') == '0' and message.protocol_version == 1 and message.request_id
                and local and message.server_id == local.get('server_id')):
            self.board.record('aircraft_v1_command', True)

    def aircraft_v2(self, message):
        value = json_sample(message, 'updated_at')
        header = getattr(message, '_connection_header', None) or {}
        if value and value.get('version') == 2 and value.get('vehicle_id') == 'uav':
            age = max(0., rospy.Time.now().to_sec() - value['updated_at'])
            self.board.record('aircraft_v2', header.get('callerid'), time.monotonic() - age)

    def origin(self, now):
        geo = self.board.recent('geographic', now, NAV_STATUS_AGE) or {}
        origin = geo.get('origin')
        if (geo.get('registered') is True and geo.get('datum') == 'WGS84'
                and geo.get('state') in ('ready', 'degraded') and isinstance(origin, dict)):
            return origin
        return None

    def fresh_transform(self, frame, source):
        try:
            transform = self.tf.lookup_transform(frame, source, rospy.Time(0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return None
        return transform if recent_stamp(transform.header.stamp.to_sec(), 1.5) else None

    def display_frame(self, now):
        return self.board.display_frame(now, self.fresh_transform('map', 'odom') is not None)

    def point_in(self, point, frame):
        if point.header.frame_id == frame:
            return point
        transform = self.fresh_transform(frame, point.header.frame_id)
        return tf2_geometry_msgs.do_transform_point(point, transform) if transform else None

    def wgs84(self, point, now):
        origin = self.origin(now)
        if origin is None:
            return {}
        converted = self.point_in(point, 'map')
        if converted is None:
            return {}
        try:
            latitude, longitude, _ = enu_to_geodetic([converted.point.x, converted.point.y, 0.], origin)
            return {'latitude': latitude, 'longitude': longitude}
        except (ValueError, KeyError, TypeError, ArithmeticError):
            return {}

    @staticmethod
    def point(frame, x, y, z=0.):
        value = PointStamped()
        value.header.frame_id = frame
        value.point.x, value.point.y, value.point.z = x, y, z
        return value

    def waypoint(self, message):
        p = message.pose.position
        if (message.header.frame_id not in ('odom', 'map') or not finite(p.x, p.y, p.z)
                or not recent_stamp(message.header.stamp.to_sec(), 2.)):
            return
        point = self.point(message.header.frame_id, p.x, p.y, p.z)
        now = time.monotonic()
        with self.lock:
            geographic = self.wgs84(point, now)
            text = coordinates(geographic) if geographic else ''
            result = text or ('等待地图 TF' if self.origin(now) is not None else WAYPOINT_PROMPT)
            self.waypoint_status, self.waypoint_coords = f'Waypoint：{result}', text
        print(f'Waypoint | 经纬度：{result}', flush=True)
        if text and self.waypoint_selected is not None:
            self.waypoint_selected(geographic, text)

    def _goal_blocked_locked(self, reason, now):
        self.manual_feedback = reason, now
        print(f'RViz手动目标未发送 · {reason}', flush=True)

    def _cancel_pending_goal_locked(self, reason):
        if self.pending_goal is not None:
            self.pending_goal = None
            self._goal_blocked_locked(reason, time.monotonic())

    def cancel_pending_goal(self):
        with self.lock:
            self._cancel_pending_goal_locked('已取消等待发送的目标')

    def set_calibration_active(self, active):
        with self.lock:
            self.calibration_active = active
            if active:
                self._cancel_pending_goal_locked('正在地理配准，等待配准动作结束')

    def manual_goal(self, message):
        if not recent_stamp(message.header.stamp.to_sec(), 1.):
            return
        p, q = message.pose.position, message.pose.orientation
        if not finite(p.x, p.y, p.z, q.x, q.y, q.z, q.w) or abs(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w - 1.) > .01:
            return
        with self.lock:
            self.pending_goal = None
            self._dispatch_goal_locked(message, time.monotonic(), may_wait=True)

    def _dispatch_goal_locked(self, message, now, may_wait=False):
        if self.board.local_target() is not None:
            self._goal_blocked_locked('正在执行本地 WGS84 任务', now)
            return
        if self.calibration_active:
            self._goal_blocked_locked('正在地理配准，等待配准动作结束', now)
            return
        frame = self.board.planner_frame(now)
        status = self.board.recent('action', now, NAV_STATUS_AGE)
        geo = self.board.recent('geographic', now, NAV_STATUS_AGE)
        reason = manual_goal_block(frame, self.board.recent('driver', now, NAV_STATUS_AGE),
                                   self.board.recent('mux', now, NAV_STATUS_AGE), status, geo,
                                   [self.board.recent(key, now) for key in ('v1', 'v2')],
                                   self.desktop_goals,
                                   runtime=self.board.recent('runtime', now, NAV_STATUS_AGE))
        if reason is None:
            reported = dict(status)
            if any(now - seen <= GOAL_STATUS_AGE and goal not in reported and goal not in self.desktop_goals
                   for goal, (_, seen) in self.action_goals.items()):
                reason = 'move_base 有其他控制端的目标，当前目标优先'
        configured = self.board.recent('configured_frame', now, NAV_STATUS_AGE)
        if reason is None and configured != frame:
            reason = '导航坐标系尚未确认或已切换，请重新点选'
        if reason is None and message.header.frame_id != frame:
            transform = (self.fresh_transform(frame, 'map')
                         if message.header.frame_id == 'map' and self.display_frame(now) == 'map' else None)
            if transform is None:
                reason = '导航坐标系尚未确认或已切换，请重新点选'
            else:
                message = tf2_geometry_msgs.do_transform_pose(message, transform)
        if reason is None and not self.goal_pub.impl.has_connection(ACTION):
            reason = '等待 move_base 目标接口'
        if reason is not None:
            self._goal_blocked_locked(reason, now)
            return
        if self.board.recent('action', now, GOAL_STATUS_AGE) is None:
            if may_wait:
                self.pending_goal = message, now + GOAL_WAIT
                self.manual_feedback = '等待下一帧 move_base 状态', now
            else:
                self._goal_blocked_locked('未收到新的 move_base 状态，请重新点选', now)
            return
        request = MoveBaseActionGoal()
        request.header.stamp = request.goal_id.stamp = rospy.Time.now()
        request.goal_id.id = 'slicer_rtk_desktop-' + uuid.uuid4().hex
        request.goal.target_pose = deepcopy(message)
        request.goal.target_pose.header.stamp = request.header.stamp
        self.goal_pub.publish(request)
        self.goal_id, self.goal_sent = request.goal_id.id, now
        self.desktop_goals[self.goal_id] = now
        p = message.pose.position
        point = self.point(frame, p.x, p.y, p.z)
        geographic = self.wgs84(point, now)
        self.manual_target = point, geographic
        self.manual_feedback = None
        print(f'RViz手动目标已发送 · {coordinates(geographic) if geographic else "待配准或等待 TF"}',
              flush=True)

    def destination(self, now):
        target = task_destination(self.board, now)
        if target:
            return target
        runtime = self.board.recent('runtime', now) or {}
        if runtime.get('busy'):
            return None
        with self.lock:
            statuses = self.board.recent('action', now, NAV_STATUS_AGE)
            if self.goal_id is not None and (statuses is not None or now - self.goal_sent <= 2.):
                point, geographic = self.manual_target
                if not geographic:
                    geographic = self.wgs84(point, now)
                    self.manual_target = point, geographic
                return coordinates(geographic) if geographic else '待配准或等待 TF'
            active = [item for item, code in statuses or () if code in ACTIVE_ACTION_STATES]
            message = next((self.action_goals[item][0] for item in active if item in self.action_goals), None)
        if message is not None:
            p = message.pose.position
            geographic = self.wgs84(self.point(message.header.frame_id, p.x, p.y, p.z), now)
            return coordinates(geographic) if geographic else '待配准或等待 TF'
        if active:
            return '等待目标坐标'
        failed_target = self.board.recent('wgs84_failure', now, 30.)
        if failed_target is None and runtime.get('phase') in ('BLOCKED', 'FAILED'):
            failed_target = runtime.get('target')
        if isinstance(failed_target, dict):
            text = coordinates(failed_target)
            return text if text != '—' else None
        return None

    def view(self, now):
        current = self.board.recent('map_odom', now, 1.5)
        frame = 'map'
        if current is None:
            current = self.board.recent('odom', now, 1.5)
            frame = 'odom'
        self.board.record('own', self.wgs84(self.point(frame, *current), now) if current else {}, now)
        self.board.record('destination', self.destination(now), now)
        with self.lock:
            if self.pending_goal is not None and now > self.pending_goal[1]:
                self._cancel_pending_goal_locked('未收到新的 move_base 状态，请重新点选')
            feedback = self.manual_feedback
            waypoint, waypoint_coords = self.waypoint_status, self.waypoint_coords
        view = self.board.view(now)
        view['waypoint'], view['waypoint_coords'] = waypoint, waypoint_coords
        view['feedback'] = feedback[0] if feedback and now-feedback[1] < 5. else ''
        return view
