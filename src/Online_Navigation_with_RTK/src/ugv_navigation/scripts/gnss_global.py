#!/usr/bin/env python3
"""ENU registration/observation adapter; robot_localization owns the estimator.

No serial, NTRIP, motion generation or TF broadcaster. All targets still use
move_base. A real odom reset cancels targets and invokes the existing hold CLI.
"""
from collections import deque
import json
import math
from pathlib import Path
import subprocess
import sys
import threading

import rospy
from actionlib_msgs.msg import GoalID
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from robot_localization.srv import SetPose, SetPoseRequest
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse
import yaml

# Catkin devel relays execute the source file; install uses installed siblings.
sys.path.insert(0, str(Path(__file__).resolve().parent))

from cup_gnss.enu import to_enu
from geographic_core import (base_from_antenna, fit_alignment,
                             interpolate_odom, transform_xy, wrap)
from geographic_state import antenna_xy, finite, horizontal_covariance, odom_reset, qualified_fix


def pose_record(message):
    p, q = message.pose.pose.position, message.pose.pose.orientation
    norm = sum(v*v for v in (q.x, q.y, q.z, q.w))
    if not .99 <= norm <= 1.01 or not all(finite(v) for v in (p.x, p.y, q.x, q.y, q.z, q.w)):
        raise ValueError('invalid_odom_pose')
    return dict(stamp=message.header.stamp.to_sec(), x=p.x, y=p.y,
                yaw=math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)),
                frame=message.header.frame_id, child=message.child_frame_id,
                vx=message.twist.twist.linear.x, vy=message.twist.twist.linear.y,
                wz=message.twist.twist.angular.z)


class GeographicLocalization:
    def __init__(self):
        self.lock = threading.RLock()
        self.options = rospy.get_param('~')
        self.site = yaml.safe_load(Path(self.options['site_config']).read_text())
        source = yaml.safe_load(Path(self.options['source_config']).read_text())
        self.lever = antenna_xy(self.site, source)
        self.file = Path(self.options['session_file'])
        self.origin = None
        configured = self.site.get('origin', {})
        if all(finite(configured.get(k)) for k in ('latitude', 'longitude', 'altitude_ellipsoid_m')):
            to_enu(configured['latitude'], configured['longitude'], configured['altitude_ellipsoid_m'], configured)
            self.origin = dict(configured)
        elif any(configured.get(k) is not None for k in ('latitude', 'longitude', 'altitude_ellipsoid_m')):
            raise ValueError('partial_origin_configuration')
        if self.file.is_file():
            saved = json.loads(self.file.read_text())
            if saved.get('datum') != self.site.get('datum'):
                raise ValueError('saved_session_datum_differs')
            if self.origin is not None and self.origin != saved.get('origin'):
                raise ValueError('saved_session_origin_differs; select a new session_file for a new task')
            self.origin = saved.get('origin')
        self.datum = self.site.get('datum')
        self.registered, self.collecting = False, False
        self.generation, self.alignment = 0, None
        self.odoms, self.globals, self.pairs = deque(maxlen=600), deque(maxlen=600), []
        self.pending, self.qualities, self.done = {}, {}, deque(maxlen=1000)
        self.last_fix_stamp_ns = 0
        self.latest, self.qualified_stamp, self.reason = None, None, 'registration_required'
        self.previous = None
        self.observation_count, self.rejected_count = 0, 0
        self.correction_reference, self.rebuild_recommended = None, False
        self.display_params_at, self.depth_enabled, self.grid_info = -1, False, {}
        self.motion_pub = rospy.Publisher('/geographic/motion', Odometry, queue_size=20)
        self.gnss_pub = rospy.Publisher('/gnss/odometry', Odometry, queue_size=10)
        self.status_pub = rospy.Publisher('/geographic/status', String, queue_size=10, latch=True)
        self.cancel_pub = rospy.Publisher('/ugv/move_base/cancel', GoalID, queue_size=1)
        self.subscribers = [
            rospy.Subscriber('/odom', Odometry, self.odom, queue_size=100),
            rospy.Subscriber('/global/odom', Odometry, self.global_odom, queue_size=100),
            rospy.Subscriber('/gnss/fix', NavSatFix, self.fix, queue_size=20),
            rospy.Subscriber('/gnss/fix_quality', DiagnosticArray, self.quality, queue_size=20)]
        self.services = [
            rospy.Service('/geographic/begin_registration', Trigger, self.begin),
            rospy.Service('/geographic/finish_registration', Trigger, self.finish)]
        self.timer = rospy.Timer(rospy.Duration(.1), self.tick)

    def save(self, reason=''):
        self.file.parent.mkdir(parents=True, exist_ok=True)
        value = dict(datum=self.datum, origin=self.origin, alignment=self.alignment,
                     generation=self.generation, invalid_reason=reason,
                     antenna_in_base_link_xy_m=self.lever,
                     timestamp_semantics='GNSS host receipt/adapter time; not PPS measurement time')
        temporary = self.file.with_suffix(self.file.suffix+'.tmp')
        temporary.write_text(json.dumps(value, indent=2, allow_nan=False)+'\n')
        temporary.replace(self.file)

    def invalidate(self, reason):
        active = self.registered or self.collecting
        self.registered = self.collecting = False
        self.alignment, self.latest = None, None
        self.last_fix_stamp_ns = 0
        self.reason = reason
        self.pairs.clear()
        self.odoms.clear()
        self.globals.clear()
        self.pending.clear()
        self.qualities.clear()
        if not active:
            return
        self.generation += 1
        try:
            self.save(reason)
        except OSError as error:
            self.reason = reason+'; persist_failed: '+str(error)
        self.cancel_pub.publish(GoalID())
        # This is the existing ownership protocol, not another velocity output.
        # Only a detected reset invokes it, once, with no release or retry.
        threading.Thread(target=self.hold, daemon=True).start()

    @staticmethod
    def hold():
        try:
            result = subprocess.run(['rosrun', 'ugv_slicer_control', 'control.py', 'hold'],
                                    capture_output=True, text=True, timeout=5.)
            if result.returncode:
                rospy.logerr('Existing hold failed after odom reset: %s', result.stderr.strip())
        except (OSError, subprocess.TimeoutExpired) as error:
            rospy.logerr('Existing hold unavailable after odom reset: %s', error)

    def odom(self, message):
        try:
            row = pose_record(message)
        except ValueError as error:
            with self.lock:
                self.invalidate(str(error))
            return
        with self.lock:
            reason = odom_reset(self.previous, row)
            self.previous = row
            if reason:
                self.invalidate(reason)
            if row['frame'] != 'odom' or row['child'] != 'base_link':
                self.reason = 'expected_odom_base_link'
                return
            if self.odoms and row['stamp'] <= self.odoms[-1]['stamp']:
                return
            self.odoms.append(row)
            if self.registered:
                # EKF fuses twist only. Its original pose is deliberately ignored.
                self.motion_pub.publish(message)

    def global_odom(self, message):
        try:
            row = pose_record(message)
        except ValueError:
            return
        if row['frame'] != 'map' or row['child'] != 'base_link':
            return
        with self.lock:
            if self.globals and row['stamp'] <= self.globals[-1]['stamp']:
                return
            self.globals.append(row)
            if self.registered and self.odoms:
                try:
                    local = interpolate_odom(list(self.odoms), row['stamp'], self.options['max_odom_gap_s'])
                except ValueError:
                    return
                yaw = wrap(row['yaw']-local['yaw'])
                c, s = math.cos(yaw), math.sin(yaw)
                current = [row['x']-c*local['x']+s*local['y'], row['y']-s*local['x']-c*local['y']]
                if self.correction_reference is None:
                    self.correction_reference = current
                elif math.hypot(*(a-b for a, b in zip(current, self.correction_reference))) > .5:
                    self.rebuild_recommended = True

    def fix(self, message):
        with self.lock:
            self.pending[message.header.stamp.to_nsec()] = message

    def quality(self, message):
        for status in message.status:
            if status.name == 'cup_gnss/receiver':
                with self.lock:
                    self.qualities[message.header.stamp.to_nsec()] = {pair.key: pair.value for pair in status.values}

    def begin(self, _request):
        with self.lock:
            if self.datum != 'WGS84':
                return TriggerResponse(False, 'set the verified datum in gnss_site.yaml; no relabelling')
            self.registered, self.collecting = False, True
            self.generation += 1
            self.pairs.clear()
            self.latest, self.alignment = None, None
            self.reason = 'collecting_stationary_endpoints_and_motion'
            return TriggerResponse(True, json.dumps(dict(generation=self.generation, origin=self.origin)))

    def finish(self, _request):
        with self.lock:
            if not self.collecting:
                return TriggerResponse(False, 'registration_not_collecting')
            try:
                moving = [i for i, pair in enumerate(self.pairs)
                          if abs(pair['odom'].get('vx', 0.)) > .02 or abs(pair['odom'].get('wz', 0.)) > .02]
                if not moving:
                    raise ValueError('no_motion_between_stationary_windows')
                start, end = self.pairs[:moving[0]], self.pairs[moving[-1]+1:]
                if not start or not end:
                    raise ValueError('need_stationary_start_and_end_GNSS_windows')
                alignment = fit_alignment(start+end, self.lever,
                                          self.options['min_alignment_displacement_m'],
                                          self.options['max_alignment_residual_m'])
                current = self.odoms[-1]
                x, y = transform_xy(current['x'], current['y'], alignment['yaw'], *alignment['translation'])
                seed = PoseWithCovarianceStamped()
                seed.header.frame_id, seed.header.stamp = 'map', rospy.Time.now()
                seed.pose.pose.position.x, seed.pose.pose.position.y = x, y
                yaw = wrap(alignment['yaw']+current['yaw'])
                seed.pose.pose.orientation.z, seed.pose.pose.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
                for index in (0, 7):
                    seed.pose.covariance[index] = self.options['gnss_position_std_m']**2
                for index in (14, 21, 28):
                    seed.pose.covariance[index] = 1e-6
                seed.pose.covariance[35] = alignment['heading_std_rad']**2
                self.alignment = alignment
                try:
                    self.save()
                except OSError as error:
                    self.alignment = None
                    self.collecting = False
                    self.reason = 'persist_failed: '+str(error)
                    return TriggerResponse(False, self.reason)
                rospy.wait_for_service('/global_localization/set_pose', timeout=3.)
                rospy.ServiceProxy('/global_localization/set_pose', SetPose)(SetPoseRequest(pose=seed))
                self.globals.clear()
                self.pending.clear()
                self.qualities.clear()
                self.latest = None
                self.registered, self.collecting = True, False
                self.reason, self.correction_reference, self.rebuild_recommended = '', None, False
                return TriggerResponse(True, json.dumps(alignment, allow_nan=False))
            except (ValueError, IndexError, rospy.ROSException, rospy.ServiceException) as error:
                self.reason = str(error)
                return TriggerResponse(False, self.reason)

    def process_fix(self, message, quality, now):
        fix = dict(stamp=message.header.stamp.to_sec(), stamp_ns=message.header.stamp.to_nsec(),
                   frame=message.header.frame_id, status=message.status.status,
                   latitude=message.latitude, longitude=message.longitude, altitude=message.altitude,
                   covariance_type=message.position_covariance_type, covariance=list(message.position_covariance))
        quality = dict(quality, stamp=fix['stamp'], stamp_ns=fix['stamp_ns'])
        reason = qualified_fix(fix, quality, now, self.options['max_fix_age_s'])
        if reason:
            self.reason = reason
            if self.latest is not None:
                self.latest = dict(self.latest, fresh_rtk_usable=False, rejected_reason=reason)
            self.rejected_count += 1
            return True
        try:
            odom = interpolate_odom(list(self.odoms), fix['stamp'], self.options['max_odom_gap_s'])
        except ValueError:
            return False  # wait for the bracketing odom sample, never extrapolate
        nearest = min(self.odoms, key=lambda row: abs(row['stamp']-fix['stamp']))
        odom.update(vx=nearest['vx'], wz=nearest['wz'])
        self.qualified_stamp = fix['stamp']
        if self.origin is None and self.collecting:
            self.origin = dict(latitude=fix['latitude'], longitude=fix['longitude'], altitude_ellipsoid_m=fix['altitude'])
            try:
                self.save()
            except OSError as error:
                self.collecting = False
                self.reason = 'persist_failed: '+str(error)
                return True
        if self.origin is None or self.datum != 'WGS84':
            return True
        enu = list(to_enu(fix['latitude'], fix['longitude'], fix['altitude'], self.origin))
        if self.collecting:
            self.pairs.append(dict(stamp=fix['stamp'], odom=odom, enu=enu))
        if not self.registered:
            return True
        yaw = self.alignment['yaw']+odom['yaw']
        predicted = None
        try:
            predicted = interpolate_odom(list(self.globals), fix['stamp'], self.options['max_odom_gap_s'])
            yaw = predicted['yaw']
        except ValueError:
            self.reason = 'waiting_for_global_prediction_at_GNSS_stamp'
            return False  # never bypass the innovation bound due to ordinary lag
        xy = base_from_antenna(enu, yaw, self.lever)
        if predicted and math.hypot(xy[0]-predicted['x'], xy[1]-predicted['y']) > self.options['max_gnss_innovation_m']:
            self.reason = 'GNSS innovation exceeds configured bound; origin retained, check or re-register'
            self.rejected_count += 1
            if self.latest is not None:
                self.latest = dict(self.latest, fresh_rtk_usable=False, rejected_reason=self.reason)
            return True
        covariance, assumed = horizontal_covariance(fix, self.options['gnss_position_std_m'])
        # Heading uncertainty projects the antenna lever into horizontal variance.
        lever_variance = sum(v*v for v in self.lever)*self.alignment['heading_std_rad']**2
        result = Odometry()
        result.header.stamp, result.header.frame_id, result.child_frame_id = message.header.stamp, 'map', 'base_link'
        result.pose.pose.position.x, result.pose.pose.position.y = xy
        result.pose.pose.orientation.w = 1.
        result.pose.covariance[0], result.pose.covariance[1] = covariance[0]+lever_variance, covariance[1]
        result.pose.covariance[6], result.pose.covariance[7] = covariance[2], covariance[3]+lever_variance
        for index in (14, 21, 28, 35):
            result.pose.covariance[index] = 1e6  # not fused as pose/orientation
        self.gnss_pub.publish(result)
        self.latest = dict(stamp=fix['stamp'], base_xy=xy, gga_quality=4, fresh_rtk_usable=True,
                           covariance_assumed=assumed, latitude=fix['latitude'], longitude=fix['longitude'],
                           altitude_ellipsoid_m=fix['altitude'], session=quality.get('session'),
                           reference='original GNSS minus one horizontal antenna lever',
                           covariance_xy=[result.pose.covariance[i] for i in (0, 1, 6, 7)])
        self.observation_count += 1
        self.reason = ''
        return True

    def tick(self, _event):
        with self.lock:
            now = rospy.Time.now().to_sec()
            if now-self.display_params_at >= 1.:
                plugins = rospy.get_param('/ugv/move_base/local_costmap/plugins', [])
                self.depth_enabled = any(item.get('name') == 'depth_layer' for item in plugins)
                grid = rospy.get_param('/ugv/move_base/global_costmap', {})
                self.grid_info = {key: grid.get(key) for key in ('global_frame', 'width', 'height', 'origin_x', 'origin_y', 'resolution')}
                self.display_params_at = now
            for key, message in list(self.pending.items()):
                if key in self.done or key <= self.last_fix_stamp_ns:
                    self.pending.pop(key, None)
                    self.qualities.pop(key, None)
                    continue
                quality = self.qualities.get(key)
                if quality is not None and self.process_fix(message, quality, now):
                    self.pending.pop(key, None)
                    self.qualities.pop(key, None)
                    self.done.append(key)
                    self.last_fix_stamp_ns = key
                elif now-message.header.stamp.to_sec() > self.options['max_fix_age_s']:
                    self.pending.pop(key, None)
            self.qualities = {k: v for k, v in self.qualities.items() if now-k/1e9 <= self.options['max_fix_age_s']}
            fresh = (self.latest is not None and self.latest['fresh_rtk_usable'] and
                     0 <= now-self.latest['stamp'] <= self.options['max_fix_age_s'])
            if self.datum != 'WGS84':
                state, reason = 'unconfigured', 'verified WGS84 datum required'
            elif self.collecting:
                state, reason = 'collecting', self.reason
            elif self.registered:
                state, reason = ('ready', '') if fresh and not self.reason else ('degraded', self.reason or 'RTK unavailable/stale')
            else:
                state, reason = 'invalid', self.reason
            value = dict(state=state, reason=reason, status_stamp=now, registered=self.registered,
                         datum=self.datum, origin=self.origin, alignment=self.alignment,
                         latest_observation=self.latest, generation=self.generation,
                         observation_count=self.observation_count, rejected_count=self.rejected_count,
                         registration_samples=len(self.pairs), qualified_stamp=self.qualified_stamp,
                         antenna_in_base_link_xy_m=self.lever, physical_reference='base_link',
                         timestamp_semantics='host receipt/adapter time, not PPS measurement time',
                         depth='enabled/candidate' if self.depth_enabled else 'disabled/not required',
                         gmapping='disabled/not required',
                         global_grid=self.grid_info,
                         tf_owners={'map->odom': '/global_localization/ekf', 'odom->base_link': '/slicer_ekf'},
                         costmap_rebuild_recommended=self.rebuild_recommended)
            self.status_pub.publish(String(data=json.dumps(value, allow_nan=False)))


if __name__ == '__main__':
    rospy.init_node('geographic')
    node = GeographicLocalization()
    rospy.spin()
