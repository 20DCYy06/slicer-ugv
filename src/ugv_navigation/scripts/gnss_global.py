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
import time

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
from geographic_core import (TrajectoryEvidence, base_from_antenna, fit_alignment, fit_trajectory_alignment,
                             interpolate_odom, transform_xy, wrap)
from geographic_goal_core import quiet_odom_tail
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
        self.registration_mode, self.registration_started, self.registered_stamp = None, None, None
        self.trajectory, self.resume_anchor = None, False
        self.trajectory_evidence = TrajectoryEvidence()
        self.paired_rtk_reason = 'waiting_for_qualified_rtk_odom_pair'
        fit_trajectory_alignment([], self.lever, **self.options.get('automatic_registration', {}))
        self.odoms, self.globals = deque(maxlen=600), deque(maxlen=600)
        self.pairs = deque(maxlen=3600)
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
            rospy.Service('/geographic/finish_registration', Trigger, self.finish),
            rospy.Service('/geographic/abort_registration', Trigger, self.abort),
            rospy.Service('/geographic/auto_begin_registration', Trigger, self.auto_begin),
            rospy.Service('/geographic/auto_check_registration', Trigger, self.auto_check),
            rospy.Service('/geographic/auto_finish_registration', Trigger, self.auto_finish),
            rospy.Service('/geographic/auto_suspend_registration', Trigger, self.auto_suspend),
            rospy.Service('/geographic/auto_abort_registration', Trigger, self.auto_abort)]
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
        had_evidence = active or self.registration_mode == 'automatic'
        self.clear_collection()
        self.registered = False
        self.registered_stamp = self.qualified_stamp = None
        self.paired_rtk_reason = 'waiting_for_qualified_rtk_odom_pair'
        self.alignment, self.latest = None, None
        self.last_fix_stamp_ns = 0
        self.reason = reason
        self.odoms.clear()
        self.globals.clear()
        self.pending.clear()
        self.qualities.clear()
        self.done.clear()
        if not had_evidence:
            return
        self.generation += 1
        try:
            self.save(reason)
        except OSError as error:
            self.reason = reason+'; persist_failed: '+str(error)
        if not active:
            return
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
            row['received'] = time.monotonic()
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
            if self.registered_stamp is not None and row['stamp'] < self.registered_stamp:
                return
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
            if not self.registered and self.registration_mode == 'automatic':
                return TriggerResponse(False, 'automatic_registration_in_progress')
            self.clear_collection()
            self.registered, self.collecting = False, True
            self.registration_mode = 'manual'
            self.generation += 1
            self.registered_stamp = None
            self.latest, self.alignment = None, None
            self.reason = 'collecting_stationary_endpoints_and_motion'
            return TriggerResponse(True, json.dumps(dict(generation=self.generation, origin=self.origin)))

    def finish(self, _request):
        with self.lock:
            if self.registration_mode == 'automatic':
                return TriggerResponse(False, 'automatic_registration_in_progress')
            if not self.collecting:
                return TriggerResponse(False, 'registration_not_collecting')
            try:
                moving = [i for i, pair in enumerate(self.pairs)
                          if abs(pair['odom'].get('vx', 0.)) > .02 or abs(pair['odom'].get('wz', 0.)) > .02]
                if not moving:
                    raise ValueError('no_motion_between_stationary_windows')
                pairs = list(self.pairs)
                start, end = pairs[:moving[0]], pairs[moving[-1]+1:]
                if not start or not end:
                    raise ValueError('need_stationary_start_and_end_GNSS_windows')
                alignment = fit_alignment(start+end, self.lever,
                                          self.options['min_alignment_displacement_m'],
                                          self.options['max_alignment_residual_m'])
                return self.commit(alignment)
            except (ValueError, IndexError) as error:
                self.reason = str(error)
                return TriggerResponse(False, self.reason)

    def clear_collection(self):
        self.collecting = False
        self.registration_mode = self.registration_started = None
        self.trajectory, self.resume_anchor = None, False
        self.trajectory_evidence = TrajectoryEvidence()
        self.pairs.clear()

    def stop_collection(self, mode, keep=False):
        with self.lock:
            if self.registered or self.registration_mode is None:
                return TriggerResponse(True, 'no_pending_registration')
            if self.registration_mode != mode:
                return TriggerResponse(False, self.registration_mode+'_registration_in_progress')
            if keep:
                self.collecting = False
                self.reason = 'automatic_registration_suspended'
            else:
                self.clear_collection()
                self.reason = 'registration_aborted'
            return TriggerResponse(True, self.reason)

    def abort(self, _request):
        return self.stop_collection('manual')

    def auto_abort(self, _request):
        return self.stop_collection('automatic')

    def auto_suspend(self, _request):
        return self.stop_collection('automatic', keep=True)

    def auto_begin(self, _request):
        with self.lock:
            if self.datum != 'WGS84':
                return TriggerResponse(False, 'set the verified datum in gnss_site.yaml; no relabelling')
            if not self.registered:
                if self.collecting and self.registration_mode != 'automatic':
                    return TriggerResponse(False, 'manual_registration_in_progress')
                if not self.collecting:
                    if self.registration_mode == 'automatic':
                        self.resume_anchor = bool(self.pairs)
                    else:
                        self.clear_collection()
                        self.generation += 1
                    self.registration_mode = 'automatic'
                    self.registration_started = rospy.Time.now().to_sec()
                    self.collecting = True
                    self.reason = 'collecting_paired_trajectory'
            payload = dict(generation=self.generation, origin=self.origin, registered=self.registered)
            return TriggerResponse(True, json.dumps(payload, allow_nan=False))

    def paired_rtk_problem(self, now, since=None):
        if self.paired_rtk_reason:
            return self.paired_rtk_reason
        stamp = self.qualified_stamp
        if stamp is None or not 0 <= now-stamp <= self.options['max_fix_age_s']:
            return 'qualified_RTK_odom_pair_stale'
        if since is not None and stamp < since:
            return 'waiting_for_qualified_RTK_odom_pair_after_start'
        if not self.odoms or not 0 <= now-self.odoms[-1]['stamp'] <= self.options['max_odom_gap_s']:
            return 'local_odom_stale'
        return ''

    def trajectory_fit(self):
        stamp = self.pairs[-1]['stamp'] if self.pairs else None
        if self.trajectory is None or self.trajectory[0] != stamp:
            alignment, progress = fit_trajectory_alignment(
                self.pairs, self.lever, evidence=self.trajectory_evidence,
                **self.options.get('automatic_registration', {}))
            self.trajectory = stamp, alignment, progress
        return self.trajectory[1:]

    def auto_candidate(self, now):
        if not self.collecting or self.registration_mode != 'automatic':
            return None, 'automatic_registration_not_collecting'
        reason = self.paired_rtk_problem(now, self.registration_started)
        if reason:
            return None, reason
        alignment, progress = self.trajectory_fit()
        return alignment, progress['reason']

    def auto_check(self, _request):
        with self.lock:
            alignment, reason = self.auto_candidate(rospy.Time.now().to_sec())
            return (TriggerResponse(True, json.dumps(alignment, allow_nan=False)) if alignment
                    else TriggerResponse(False, reason))

    def auto_finish(self, _request):
        with self.lock:
            if self.registered and self.registration_mode == 'automatic':
                return TriggerResponse(True, json.dumps(self.alignment, allow_nan=False))
            now = rospy.Time.now().to_sec()
            alignment, reason = self.auto_candidate(now)
            if not alignment:
                return TriggerResponse(False, reason)
            rows = [dict(row, speed=math.hypot(row['vx'], row['vy']), angular_speed=row['wz'])
                    for row in self.odoms]
            stop = quiet_odom_tail(rows, time.monotonic(), now,
                                   window=self.options.get('stop_window_s', 1.),
                                   max_age=min(.5, self.options['max_odom_gap_s']))
            if not stop['confirmed']:
                return TriggerResponse(False, stop['reason'])
            if not stop['window_start_stamp'] <= self.qualified_stamp <= stop['window_end_stamp']:
                return TriggerResponse(False, 'waiting_for_qualified_RTK_in_stopped_window')
            return self.commit(alignment)

    def commit(self, alignment):
        set_pose = '/global_localization/set_pose'
        automatic = self.registration_mode == 'automatic'
        try:
            rospy.wait_for_service(set_pose, timeout=.25 if automatic else 3.)
            self.alignment = alignment
            self.save()
            seed = PoseWithCovarianceStamped()
            seed.header.frame_id, seed.header.stamp = 'map', rospy.Time.now()
            if automatic:
                reason = self.paired_rtk_problem(seed.header.stamp.to_sec(), self.registration_started)
                if reason:
                    self.alignment = None
                    return TriggerResponse(False, reason)
            current = self.odoms[-1]
            x, y = transform_xy(current['x'], current['y'], alignment['yaw'], *alignment['translation'])
            seed.pose.pose.position.x, seed.pose.pose.position.y = x, y
            yaw = wrap(alignment['yaw']+current['yaw'])
            seed.pose.pose.orientation.z, seed.pose.pose.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
            for index in (0, 7):
                seed.pose.covariance[index] = self.options['gnss_position_std_m']**2
            for index in (14, 21, 28):
                seed.pose.covariance[index] = 1e-6
            seed.pose.covariance[35] = alignment['heading_std_rad']**2
            rospy.ServiceProxy(set_pose, SetPose)(SetPoseRequest(pose=seed))
        except OSError as error:
            self.alignment = None
            self.clear_collection()
            self.reason = 'persist_failed: '+str(error)
            return TriggerResponse(False, self.reason)
        except (rospy.ROSException, rospy.ServiceException) as error:
            self.alignment = None
            self.reason = 'global_EKF_seed_failed: '+str(error)
            if automatic:
                self.clear_collection()
            return TriggerResponse(False, self.reason)
        self.globals.clear()
        self.pending.clear()
        self.qualities.clear()
        self.latest = self.qualified_stamp = None
        self.registered_stamp = seed.header.stamp.to_sec()
        self.paired_rtk_reason = 'waiting_for_post_registration_pair'
        self.registered, self.collecting = True, False
        self.reason, self.correction_reference, self.rebuild_recommended = '', None, False
        return TriggerResponse(True, json.dumps(alignment, allow_nan=False))

    def process_fix(self, message, quality, now):
        fix = dict(stamp=message.header.stamp.to_sec(), stamp_ns=message.header.stamp.to_nsec(),
                   frame=message.header.frame_id, status=message.status.status,
                   latitude=message.latitude, longitude=message.longitude, altitude=message.altitude,
                   covariance_type=message.position_covariance_type, covariance=list(message.position_covariance))
        if self.registered_stamp is not None and fix['stamp'] < self.registered_stamp:
            return True
        quality = dict(quality, stamp=fix['stamp'], stamp_ns=fix['stamp_ns'])
        reason = qualified_fix(fix, quality, now, self.options['max_fix_age_s'])
        if reason:
            self.reason = self.paired_rtk_reason = reason
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
        self.paired_rtk_reason = ''
        collect = self.collecting and (self.registration_started is None or fix['stamp'] >= self.registration_started)
        if self.origin is None and collect:
            self.origin = dict(latitude=fix['latitude'], longitude=fix['longitude'], altitude_ellipsoid_m=fix['altitude'])
            try:
                self.save()
            except OSError as error:
                self.clear_collection()
                self.reason = 'persist_failed: '+str(error)
                return True
        if self.origin is None or self.datum != 'WGS84':
            return True
        enu = list(to_enu(fix['latitude'], fix['longitude'], fix['altitude'], self.origin))
        if collect:
            self.pairs.append(dict(stamp=fix['stamp'], odom=odom, enu=enu))
            if self.resume_anchor:
                self.pairs[-1]['resumed'] = True
                self.resume_anchor = False
            self.reason = ('collecting_paired_trajectory' if self.registration_mode == 'automatic'
                           else 'collecting_stationary_endpoints_and_motion')
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
            paired_reason = self.paired_rtk_problem(now, self.registration_started if self.collecting else None)
            progress = None
            if self.registration_mode == 'automatic':
                _, progress = self.trajectory_fit()
                if not self.registered and (paired_reason or not self.collecting):
                    progress = dict(progress, ready=False, reason=paired_reason if self.collecting else self.reason)
            fresh = (self.latest is not None and self.latest['fresh_rtk_usable'] and
                     0 <= now-self.latest['stamp'] <= self.options['max_fix_age_s'])
            if self.datum != 'WGS84':
                state, reason = 'unconfigured', 'verified WGS84 datum required'
            elif self.collecting:
                state, reason = 'collecting', progress['reason'] if progress is not None else self.reason
            elif self.registered:
                state, reason = ('ready', '') if fresh and not self.reason else ('degraded', self.reason or 'RTK unavailable/stale')
            else:
                state, reason = 'invalid', self.reason
            value = dict(state=state, reason=reason, status_stamp=now, registered=self.registered,
                         datum=self.datum, origin=self.origin, alignment=self.alignment,
                         latest_observation=self.latest, generation=self.generation,
                         observation_count=self.observation_count, rejected_count=self.rejected_count,
                         registration_mode=self.registration_mode, alignment_progress=progress,
                         registration_samples=len(self.pairs), qualified_stamp=self.qualified_stamp,
                         paired_rtk_usable=not paired_reason, paired_rtk_reason=paired_reason,
                         registered_stamp=self.registered_stamp,
                         global_stamp=self.globals[-1]['stamp'] if self.globals else None,
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
