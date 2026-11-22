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

from cup_gnss.enu import to_enu
from geographic_core import enu_to_geodetic
from dashboard_model import ACTIVE_ACTION_STATES, manual_goal_block, task_in_progress


ACTION = '/ugv/move_base'
# The wrapper gives each desktop its own ROS namespace.
GOAL_CLICK = 'manual_goal'
WAYPOINT_CLICK = 'waypoint'


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
        self.manual_feedback = None
        self.goal_pose = None
        self.action_goals = {}
        self.active_ids = ()
        self.active_since = 0.
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
            (ACTION + '/current_goal', PoseStamped, self.current_goal),
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
        if self.board.recent('action', time.monotonic(), 1.5) is not None:
            frame = rospy.get_param(ACTION + '/global_costmap/global_frame', None)
            if frame in ('odom', 'map'):
                self.board.record('configured_frame', frame)

    def action_status(self, message):
        if ((getattr(message, '_connection_header', None) or {}).get('callerid') != ACTION
                or not recent_stamp(message.header.stamp.to_sec(), 1.5)):
            return
        items = [(item.goal_id.id, item.status) for item in message.status_list]
        self.board.record('action', items)
        self.board.record('action_stamp', message.header.stamp.to_sec())
        with self.lock:
            active = tuple(goal for goal, status in items if status in ACTIVE_ACTION_STATES)
            if active != self.active_ids:
                self.active_ids, self.active_since = active, time.monotonic()
            if self.goal_id is not None:
                status = next((code for goal, code in items if goal == self.goal_id), None)
                if status is not None and status not in ACTIVE_ACTION_STATES:
                    self.goal_id = None
                elif status is None and time.monotonic() - self.goal_sent > 2.:
                    self.goal_id = None

    def action_goal(self, message):
        pose = message.goal.target_pose
        p = pose.pose.position
        if pose.header.frame_id in ('odom', 'map') and finite(p.x, p.y, p.z) and message.goal_id.id:
            with self.lock:
                self.action_goals[message.goal_id.id] = pose
                if len(self.action_goals) > 32:
                    self.action_goals.pop(next(iter(self.action_goals)))

    def current_goal(self, message):
        p = message.pose.position
        if message.header.frame_id in ('odom', 'map') and finite(p.x, p.y, p.z):
            with self.lock:
                self.goal_pose = message, time.monotonic()

    def task(self, key, message, timestamp):
        value = json_sample(message, timestamp)
        if value is not None:
            self.board.record(key, value)

    def current_frame(self, now):
        return self.board.planner_frame(now) or 'odom'

    def origin(self, now):
        geo = self.board.recent('geographic', now, 1.5) or {}
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
        latitude = geographic.get('latitude')
        local = self.point_in(point, 'odom')
        text = (f'{latitude:.7f}, {geographic["longitude"]:.7f}' if latitude is not None
                else '待配准或等待 TF')
        report = f'Waypoint | 经纬度：{text} | {point.header.frame_id} X: {p.x:.2f}m, Y: {p.y:.2f}m'
        if local is not None and point.header.frame_id != 'odom':
            report += f' | odom X: {local.point.x:.2f}m, Y: {local.point.y:.2f}m'
        elif local is None:
            report += ' | odom：等待 TF'
        print(report, flush=True)

    def manual_goal(self, message):
        if not recent_stamp(message.header.stamp.to_sec(), 1.):
            return
        p, q = message.pose.position, message.pose.orientation
        if not finite(p.x, p.y, p.z, q.x, q.y, q.z, q.w) or abs(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w - 1.) > .01:
            return
        now = time.monotonic()
        with self.lock:
            frame = self.board.planner_frame(now)
            status = self.board.recent('action', now, 1.5)
            if not recent_stamp(self.board.recent('action_stamp', now, .5), .5):
                status = None
            geo = self.board.recent('geographic', now, 1.5)
            reason = manual_goal_block(frame, self.board.recent('driver', now, 1.5),
                                       self.board.recent('mux', now, 1.5), status, geo,
                                       [self.board.recent(key, now, 2.5) for key in ('v1', 'v2')],
                                       self.goal_id is not None)
            configured = self.board.recent('configured_frame', now, 1.5)
            if reason is None and (message.header.frame_id != frame or configured not in (None, frame)):
                reason = '导航坐标系已切换，请重新点选'
            if reason is None and not self.goal_pub.impl.has_connection(ACTION):
                reason = '等待 move_base 目标接口'
            if reason is not None:
                self.manual_feedback = (reason, now)
                print(f'RViz手动目标未发送 · {reason}', flush=True)
                return
            request = MoveBaseActionGoal()
            request.header.stamp = request.goal_id.stamp = rospy.Time.now()
            request.goal_id.id = 'slicer_rtk_desktop-' + uuid.uuid4().hex
            request.goal.target_pose = deepcopy(message)
            request.goal.target_pose.header.stamp = request.header.stamp
            self.goal_pub.publish(request)
            self.goal_id, self.goal_sent = request.goal_id.id, now
            self.manual_feedback = None
            print(f'RViz手动目标已发送 · {frame} X: {p.x:.2f}m, Y: {p.y:.2f}m', flush=True)

    def view(self, now):
        frame = self.current_frame(now)
        current = self.board.recent('map_odom' if frame == 'map' else 'odom', now, 1.5)
        own = {}
        if current:
            point = self.point(frame, *current)
            own.update(x=current[0], y=current[1], **self.wgs84(point, now))
        self.board.record('own', own, now)
        with self.lock:
            goal, own_goal, feedback = self.goal_pose, self.goal_id, self.manual_feedback
            active_since = self.active_since
            goals = self.action_goals.copy()
        statuses = self.board.recent('action', now, 1.5) or []
        manual = own_goal is not None and any(item == own_goal and code in ACTIVE_ACTION_STATES
                                               for item, code in statuses)
        active = any(code in ACTIVE_ACTION_STATES for _, code in statuses)
        record = {}
        message = next((goals[item] for item, code in statuses if code == 1 and item in goals), None)
        if message is None and active and goal is not None and goal[1] >= active_since - .25:
            message = goal[0]
        if message is not None:
            p = message.pose.position
            point = self.point(message.header.frame_id, p.x, p.y, p.z)
            projected = self.point_in(point, frame)
            if projected is not None:
                record.update(x=projected.point.x, y=projected.point.y)
            record.update(self.wgs84(point, now))
        if not record:
            for key in ('v2', 'v1'):
                task = self.board.recent(key, now, 2.5)
                if not task_in_progress(task):
                    continue
                point = task.get('active_point') if key == 'v2' else (task.get('active') or {}).get('point')
                if key == 'v2' and isinstance(point, dict):
                    point = {'latitude': point.get('lat'), 'longitude': point.get('lon')}
                if isinstance(point, dict) and finite(point.get('latitude'), point.get('longitude')):
                    record.update(latitude=point['latitude'], longitude=point['longitude'])
                    origin = self.origin(now)
                    if origin and frame == 'map':
                        try:
                            height = point.get('altitude_ellipsoid_m')
                            x, y, _ = to_enu(point['latitude'], point['longitude'],
                                             height if finite(height) else origin['altitude_ellipsoid_m'], origin)
                            record.update(x=x, y=y)
                        except (ValueError, KeyError, TypeError, ArithmeticError):
                            pass
                    break
        self.board.record('route', {'destination': 'Point1' if active else None,
                                    'points': [record] if record else []}, now)
        self.board.record('manual', 'RViz手动目标' if manual else None, now)
        view = self.board.view(now)
        view['feedback'] = feedback[0] if feedback and now-feedback[1] < 5. else ''
        return view
