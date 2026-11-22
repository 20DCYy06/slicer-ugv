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
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String

from geographic_core import enu_to_geodetic
from dashboard_model import ACTIVE_ACTION_STATES, NAV_STATUS_AGE, coordinates, manual_goal_block


ACTION = '/ugv/move_base'
# The wrapper gives each desktop its own ROS namespace.
GOAL_CLICK = 'manual_goal'
WAYPOINT_CLICK = 'waypoint'
GOAL_STATUS_AGE = .5
GOAL_WAIT = .65


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
        self.registration = None
        self.lock = Lock()
        self.goal_id = None
        self.goal_sent = 0.
        self.manual_target = None
        self.manual_feedback = None
        self.pending_goal = None
        self.action_connection = None
        self.action_goals = {}
        self.goal_pub = rospy.Publisher(ACTION + '/goal', MoveBaseActionGoal, queue_size=1)
        self.subscribers = []
        for topic, kind, callback in (
            ('/gnss/diagnostics', DiagnosticArray, self.diagnostics),
            ('/gnss/fix_quality', DiagnosticArray, self.fix_quality),
            ('/geographic/status', String, self.geographic),
            ('/scan', LaserScan, lambda m: self.sensor('lidar', m)),
            ('/lio/odom', Odometry, lambda m: self.sensor('lio', m)),
            ('/odom', Odometry, self.local_odom),
            ('/wheel/odom', Odometry, lambda m: self.sensor('wheel_odom', m)),
            ('/global/odom', Odometry, self.global_odom),
            ('/slicer_c30d_driver/ready', Bool, lambda m: self.board.record('driver', m.data)),
            ('/cup_nav/manual_state', String, self.mux),
            (ACTION + '/global_costmap/costmap', OccupancyGrid, self.costmap),
            (ACTION + '/status', GoalStatusArray, self.action_status),
            (ACTION + '/goal', MoveBaseActionGoal, self.action_goal),
            ('/air_ground/state', String, lambda m: self.task('v1', m, 'updated_stamp')),
            ('/air_ground/v2/state', String, lambda m: self.task('v2', m, 'updated_at')),
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

    def diagnostics(self, message):
        self.quality('gnss', message)

    def fix_quality(self, message):
        self.quality('quality', message)

    def geographic(self, message):
        value = json_sample(message, 'status_stamp')
        if value is not None:
            self.board.record('geographic', value)

    def sensor(self, key, message):
        if recent_stamp(message.header.stamp.to_sec(), 2.5):
            self.board.record(key, True)

    def local_odom(self, message):
        if not recent_stamp(message.header.stamp.to_sec(), 1.):
            return
        p, speed = message.pose.pose.position, message.twist.twist
        if message.header.frame_id != 'odom' or not finite(p.x, p.y, speed.linear.x, speed.angular.z):
            return
        self.board.record('odom', (p.x, p.y))
        self.board.record('odom_still', abs(speed.linear.x) <= .02 and abs(speed.angular.z) <= .02)
        geo = self.board.recent('geographic', time.monotonic(), 1.5) or {}
        if geo.get('state') != 'collecting':
            self.registration = None
            return
        generation = geo.get('generation')
        if self.registration is None or self.registration[0] != generation:
            self.registration = generation, p.x, p.y
        _, x, y = self.registration
        self.board.record('registration_motion', {
            'moving': abs(speed.linear.x) > .02 or abs(speed.angular.z) > .02,
            'distance': math.hypot(p.x - x, p.y - y),
        })

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
        now = time.monotonic()
        with self.lock:
            if header is not self.action_connection:
                # rospy shares the header dictionary among messages on one TCPROS connection.
                self.action_connection = header
                self.board.record('configured_frame', None, now)
                self._cancel_pending_goal_locked('导航已重新连接，请重新点选')
            self.board.record('action', items, now - max(0., age))
            if self.goal_id is not None:
                status = next((code for goal, code in items if goal == self.goal_id), None)
                if status is not None and status not in ACTIVE_ACTION_STATES:
                    self.goal_id = None
                elif status is None and now - self.goal_sent > 2.:
                    self.goal_id = None
            if self.pending_goal is not None and age <= GOAL_STATUS_AGE:
                pending, deadline = self.pending_goal
                self.pending_goal = None
                if now <= deadline:
                    self._dispatch_goal_locked(pending, now)
                else:
                    self._goal_blocked_locked('未收到新的 move_base 状态，请重新点选', now)

    def action_goal(self, message):
        pose = message.goal.target_pose
        p = pose.pose.position
        if pose.header.frame_id in ('odom', 'map') and finite(p.x, p.y, p.z) and message.goal_id.id:
            with self.lock:
                self.action_goals[message.goal_id.id] = pose
                if len(self.action_goals) > 32:
                    self.action_goals.pop(next(iter(self.action_goals)))

    def task(self, key, message, timestamp):
        value = json_sample(message, timestamp)
        if value is not None:
            self.board.record(key, value)

    def origin(self, now):
        geo = self.board.recent('geographic', now, NAV_STATUS_AGE) or {}
        origin = geo.get('origin')
        if (geo.get('registered') is True and geo.get('datum') == 'WGS84'
                and geo.get('state') in ('ready', 'degraded') and isinstance(origin, dict)):
            return origin
        return None

    def point_in(self, point, frame):
        if point.header.frame_id == frame:
            return point
        try:
            transform = self.tf.lookup_transform(frame, point.header.frame_id, rospy.Time(0))
            if not recent_stamp(transform.header.stamp.to_sec(), 1.5):
                return None
            return tf2_geometry_msgs.do_transform_point(point, transform)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return None

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
        geographic = self.wgs84(point, time.monotonic())
        text = coordinates(geographic) if geographic else '待配准或等待 TF'
        print(f'Waypoint | 经纬度：{text}', flush=True)

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
        frame = self.board.planner_frame(now)
        status = self.board.recent('action', now, NAV_STATUS_AGE)
        geo = self.board.recent('geographic', now, NAV_STATUS_AGE)
        reason = manual_goal_block(frame, self.board.recent('driver', now, NAV_STATUS_AGE),
                                   self.board.recent('mux', now, NAV_STATUS_AGE), status, geo,
                                   [self.board.recent(key, now) for key in ('v1', 'v2')],
                                   self.goal_id is not None)
        configured = self.board.recent('configured_frame', now, NAV_STATUS_AGE)
        if reason is None and (message.header.frame_id != frame or configured != frame):
            reason = '导航坐标系尚未确认或已切换，请重新点选'
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
        p = message.pose.position
        point = self.point(frame, p.x, p.y, p.z)
        geographic = self.wgs84(point, now)
        self.manual_target = point, geographic
        self.manual_feedback = None
        print(f'RViz手动目标已发送 · {coordinates(geographic) if geographic else "待配准或等待 TF"}',
              flush=True)

    def destination(self, now):
        with self.lock:
            statuses = self.board.recent('action', now, NAV_STATUS_AGE)
            if self.goal_id is not None and (statuses is not None or now - self.goal_sent <= 2.):
                point, geographic = self.manual_target
                if not geographic:
                    geographic = self.wgs84(point, now)
                    self.manual_target = point, geographic
                return coordinates(geographic) if geographic else '待配准或等待 TF'
            active = [item for item, code in statuses or () if code in ACTIVE_ACTION_STATES]
            message = next((self.action_goals[item] for item in active if item in self.action_goals), None)
        if message is not None:
            p = message.pose.position
            geographic = self.wgs84(self.point(message.header.frame_id, p.x, p.y, p.z), now)
            return coordinates(geographic) if geographic else '待配准或等待 TF'
        return '等待目标坐标' if active else None

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
        view = self.board.view(now)
        view['feedback'] = feedback[0] if feedback and now-feedback[1] < 5. else ''
        return view
