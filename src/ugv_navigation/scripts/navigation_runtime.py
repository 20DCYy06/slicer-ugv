#!/usr/bin/env python3
"""Own one field navigation launch and prepare it for a pending WGS84 task."""
from dataclasses import dataclass
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
from queue import Empty, SimpleQueue
import sys
import threading
import time
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parent))

import actionlib
from actionlib_msgs.msg import GoalStatusArray
from nav_msgs.msg import OccupancyGrid, Odometry
import rosgraph
import rospy
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger, TriggerResponse
import tf2_ros

from geographic import GeographicClient
from geographic_goal_core import finite, quiet_odom_tail, status_problem
from navigation_preparation import PREPARE_ACTION, RUNTIME_TOPIC
from owned_process import OwnedProcess
from preparation_path import candidate_goals, observed_cell, observed_path
from ugv_navigation.msg import (PrepareGeographicAction, PrepareGeographicFeedback,
                                PrepareGeographicResult)

ACTION = '/ugv/move_base'
MUX = '/slicer_velocity_mux'
MUX_STATE = '/cup_nav/manual_state'
NAV_NODES = (ACTION, MUX, '/slicer_nav_arc_guard')
ACTIVE_GOALS = (0, 1, 6, 7)


@dataclass
class Preparation:
    handle: object
    target: object
    started: float
    generation: int = None
    generation_echo_until: float = 0.
    phase: str = 'WAITING_NAVIGATION'
    reason: str = 'waiting for navigation'
    canceled: bool = False
    cancellation_started: bool = False
    distance: float = 0.
    last_position: tuple = None
    progress_distance: float = 0.
    progress_at: float = None
    stopped_since: float = None
    outcome: str = None
    outcome_reason: str = ''
    moving: bool = False
    registration_changed: bool = False
    require_fresh_global: bool = False
    must_hold: bool = False


def hold_key(mux):
    return mux['mux_instance_id'], mux['takeover_generation']


def acquire_local_owner():
    key = hashlib.sha256(rosgraph.get_master_uri().encode()).hexdigest()[:16]
    lock_file = open('/tmp/slicer-navigation-' + key + '.lock', 'a')
    try:
        fcntl.flock(lock_file, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        lock_file.close()
        raise RuntimeError('a shared field navigation owner is already running on this master')
    return lock_file


class NavigationRuntime:
    def __init__(self):
        self.master = rosgraph.Master(rospy.get_name())
        self.field_launch = rospy.get_param('~field_launch')
        self.initial_mode = rospy.get_param('~initial_mode', 'odom')
        self.odom_cruise = rospy.get_param('~odom_cruise', False)
        if type(self.odom_cruise) is not bool:
            raise ValueError('odom_cruise must be boolean')
        if self.initial_mode not in ('odom', 'map'):
            raise ValueError('initial_mode must be odom or map')
        self.distances = tuple(rospy.get_param('~candidate_distances_m', [2.5, 1.8, 1.25]))
        self.max_distance = rospy.get_param('~max_distance_m', 7.)
        self.max_duration = rospy.get_param('~max_duration_s', 75.)
        self.rtk_wait = rospy.get_param('~rtk_wait_s', 15.)
        self.no_progress = rospy.get_param('~no_progress_s', 12.)
        self.switch_timeout = rospy.get_param('~switch_timeout_s', 40.)
        limits = (*self.distances, self.max_distance, self.max_duration, self.rtk_wait,
                  self.no_progress, self.switch_timeout)
        if not self.distances or not all(finite(v) and v > 0. for v in limits):
            raise ValueError('positive finite navigation preparation limits required')

        self.geo = GeographicClient(SimpleNamespace(action=ACTION, command='runtime', timeout=120.))
        self.lock = threading.RLock()
        self.request = None
        self.events = SimpleQueue()
        self.last_failure = None
        self.nav, self.mode, self.transition = None, None, None
        self.preparation = False
        self.control, self.control_call, self.owned_hold = None, None, None
        self.worker = None
        self.samples = {}
        self.emergency = False
        self.gap_takeover = False
        self.release_idle = True
        self.phase, self.reason = 'WAITING_NAVIGATION', 'starting shared navigation'
        self.startup_pending = False
        self.owner_pub = rospy.Publisher(RUNTIME_TOPIC, String, queue_size=1, latch=True)
        self.subscriptions = [
            rospy.Subscriber(ACTION + '/status', GoalStatusArray, self._action_status, queue_size=1),
            rospy.Subscriber(ACTION + '/global_costmap/costmap', OccupancyGrid, self._grid, queue_size=1),
            rospy.Subscriber(MUX_STATE, String, self._mux, queue_size=4),
            rospy.Subscriber('/slicer_c30d_driver/ready', Bool, self._driver, queue_size=1),
            rospy.Subscriber('/global/odom', Odometry, self.geo._global, queue_size=10),
            rospy.Subscriber('/cup_nav/emergency_stop', Bool, self._estop, queue_size=5),
            rospy.Subscriber('/cup_nav/manual_intent', String, self._intent, queue_size=10),
        ]
        # actionlib invokes callbacks while holding its lock. The worker
        # alone changes requests and calls back into the ActionServer.
        self.action = actionlib.ActionServer(PREPARE_ACTION, PrepareGeographicAction,
                                             lambda goal: self.events.put(('goal', goal)),
                                             lambda goal: self.events.put(('cancel', goal)),
                                             auto_start=False)
        self.map_service = rospy.Service('/geographic/navigation_map', Trigger, self._diagnostic_map)
        self.preparation_service = rospy.Service('/geographic/navigation_prepare', Trigger,
                                                 self._diagnostic_prepare)
        self.action.start()

    def _action_status(self, msg):
        if msg._connection_header.get('callerid') == ACTION:
            self.samples['action'] = time.monotonic(), bool(
                any(s.status in ACTIVE_GOALS for s in msg.status_list)), msg.header.stamp.to_sec()

    def _mux(self, msg):
        header = msg._connection_header
        if header.get('callerid') != MUX or header.get('latching', '0') != '0':
            return
        try:
            mux = json.loads(msg.data)
            if (isinstance(mux['mux_instance_id'], str) and type(mux['takeover_generation']) is int
                    and all(type(mux[k]) is bool for k in
                            ('manual_hold', 'teleop_fresh', 'emergency_stop_latched'))):
                self.samples['mux'] = time.monotonic(), mux
                if mux['emergency_stop_latched']:
                    self.emergency = True
        except (KeyError, TypeError, ValueError):
            pass

    def _driver(self, msg):
        if msg._connection_header.get('callerid') == '/slicer_c30d_driver':
            self.samples['driver'] = time.monotonic(), msg.data

    def _grid(self, msg):
        if msg._connection_header.get('callerid') == ACTION:
            self.samples['grid'] = time.monotonic(), msg

    def _estop(self, msg):
        if msg.data:
            self.emergency = True

    def _intent(self, msg):
        transition = self.transition
        if transition and transition['stage'] in ('exit', 'start'):
            try:
                if json.loads(msg.data).get('action') == 'takeover':
                    self.gap_takeover = True
            except (AttributeError, TypeError, ValueError):
                pass

    def _sample(self, name, age=.6):
        item = self.samples.get(name)
        return item[1] if item and time.monotonic() - item[0] <= age else None

    def _idle(self):
        item = self.samples.get('action')
        return (self._sample('action') is False and item is not None
                and -.1 <= rospy.Time.now().to_sec() - item[2] <= .6)

    def _nav_present(self):
        publishers, subscribers, services = self.master.getSystemState()
        return any(set(nodes).intersection(NAV_NODES)
                   for _, nodes in (*publishers, *subscribers, *services))

    def _unique_chain(self):
        pubs = dict(self.master.getSystemState()[0])
        return (pubs.get('/cmd_vel') == [MUX] and pubs.get(MUX_STATE) == [MUX]
                and pubs.get(ACTION + '/status') == [ACTION])

    def _quiet(self, since=0.):
        rows = [row for row in self.geo.observation()[2] if row['received'] >= since]
        return quiet_odom_tail(rows, time.monotonic(), rospy.Time.now().to_sec(),
                               window=self.geo.stop_window)['confirmed']

    def _local_pose(self):
        transform = self.geo.buffer.lookup_transform('odom', 'base_link', rospy.Time(0))
        if abs((rospy.Time.now() - transform.header.stamp).to_sec()) > .5:
            raise ValueError('local base TF is stale')
        p, q = transform.transform.translation, transform.transform.rotation
        return p.x, p.y, math.atan2(2. * (q.w * q.z + q.x * q.y), 1. - 2. * (q.y * q.y + q.z * q.z))

    def _status(self):
        status, received, _ = self.geo.observation()
        if not status or received is None or time.monotonic() - received > self.geo.status_age:
            return {}
        stamp = status.get('status_stamp')
        if not finite(stamp) or not -.1 <= rospy.Time.now().to_sec() - stamp <= self.geo.status_age:
            return {}
        return status

    def _global_problem(self, strict=False, generation=None):
        status, received, _ = self.geo.observation()
        now = rospy.Time.now().to_sec()
        problem = status_problem(status, received, time.monotonic(), now, generation, self.geo.status_age)
        if problem:
            return problem
        registered_stamp = status.get('registered_stamp')
        if strict:
            latest = status.get('latest_observation') or {}
            if (status.get('state') != 'ready' or latest.get('fresh_rtk_usable') is not True
                    or not finite(latest.get('stamp')) or not finite(registered_stamp)
                    or latest['stamp'] < registered_stamp
                    or not -.1 <= now - latest['stamp'] <= self.geo.fix_age):
                return 'waiting for a fresh paired FIXED observation after registration'
        global_poses = self.geo.capture_inputs()[2]
        if not global_poses or not -.1 <= now - global_poses[-1]['stamp'] <= .6:
            return 'waiting for fresh global EKF odometry'
        for parent, child in (('map', 'odom'), ('map', 'base_link')):
            try:
                transform = self.geo.buffer.lookup_transform(parent, child, rospy.Time(0))
                stamp = transform.header.stamp.to_sec()
            except tf2_ros.TransformException:
                return 'waiting for ' + parent + ' to ' + child + ' TF'
            if not -.1 <= now - stamp <= .6:
                return 'waiting for fresh ' + parent + ' to ' + child + ' TF'
            if strict and stamp < registered_stamp:
                return 'waiting for TF published after registration'
        if strict and global_poses[-1]['stamp'] < registered_stamp:
            return 'waiting for global EKF odometry after registration'
        return None

    def _accept(self, handle):
        if self.request is not None or self.transition is not None:
            handle.set_rejected(PrepareGeographicResult('FAILED', 'navigation preparation or switch already active'))
            return
        try:
            goal = handle.get_goal()
            target = json.loads(goal.target_json)
            if not isinstance(target, dict):
                raise ValueError('WGS84 target required')
            latitude, longitude = target.get('latitude'), target.get('longitude')
            if not (finite(latitude) and -90. <= latitude <= 90.
                    and finite(longitude) and -180. <= longitude <= 180.):
                raise ValueError('WGS84 latitude or longitude invalid')
            json.dumps(target, allow_nan=False)
        except (ValueError, TypeError) as error:
            handle.set_rejected(PrepareGeographicResult('FAILED', str(error)))
            return
        handle.set_accepted()
        self.last_failure = None
        self.request = Preparation(handle, target, time.monotonic())

    def _process_events(self):
        for _ in range(16):
            try:
                operation, handle = self.events.get_nowait()
            except Empty:
                break
            if operation == 'goal':
                self._accept(handle)
            elif self.request and self.request.handle == handle:
                self.request.canceled = True

    def _diagnostic_map(self, _request):
        with self.lock:
            if self.request is not None or self.transition is not None:
                return TriggerResponse(False, 'active navigation preparation or switch')
            if self.mode == 'map':
                return TriggerResponse(True, 'map navigation already running')
            problem = self._global_problem(strict=True) or self._permission()
            if problem or not self._idle() or not self._quiet():
                return TriggerResponse(False, problem or 'navigation must be idle and stopped')
            self._switch('map')
            return TriggerResponse(True, 'switching to map navigation')

    def _diagnostic_prepare(self, _request):
        with self.lock:
            if self.request is not None or self.transition is not None or self.startup_pending:
                return TriggerResponse(False, 'active navigation preparation or switch')
            if self.mode != 'odom':
                return TriggerResponse(False, 'local odom navigation must be running')
            if self.preparation:
                return TriggerResponse(True, 'odom preparation already running')
            problem = self._permission()
            if problem or not self._idle() or not self._quiet():
                return TriggerResponse(False, problem or 'navigation must be idle and stopped')
            self._switch('odom', preparation=True)
            return TriggerResponse(True, 'switching to odom preparation')

    def _permission(self, allow_owned_hold=False):
        mux = self._sample('mux')
        if self.emergency:
            return 'emergency stop is latched'
        if mux is None:
            return 'waiting for fresh Mux status'
        if self.gap_takeover or mux['teleop_fresh'] or mux['emergency_stop_latched']:
            return 'manual takeover or emergency stop'
        if mux['manual_hold'] and not (allow_owned_hold and hold_key(mux) == self.owned_hold):
            return 'external parking hold remains active'
        if mux.get('selected_source') == 'mission':
            return 'mission velocity source owns control'
        if self._sample('driver', 1.) is not True:
            return 'waiting for ready chassis driver'
        return None

    def _start(self, mode, preparation=None):
        if preparation is None:
            preparation = mode == 'odom' and not self.odom_cruise
        if self._nav_present():
            raise RuntimeError('external field navigation is present; stop it from its own terminal')
        self.samples.pop('action', None)
        self.samples.pop('grid', None)
        self.samples.pop('mux', None)
        self.nav = OwnedProcess(mode, ('roslaunch', '--sigint-timeout', '4',
                                      '--sigterm-timeout', '2', self.field_launch,
                                      'geographic:=' + ('true' if mode == 'map' else 'false'),
                                      'preparation:=' + ('true' if preparation else 'false')),
                                os.environ.copy(), log_prefix='rtk-navigation')
        self.mode, self.owned_hold = mode, None
        self.preparation = preparation
        self.startup_pending = True

    def _begin_control(self, operation, key):
        argv = ['rosrun', 'ugv_slicer_control', 'control.py', operation,
                '--expected-mux-instance', key[0], '--expected-generation', str(key[1])]
        if operation == 'hold':
            argv.append('--require-idle')
        self.control = OwnedProcess(operation, argv, os.environ.copy(), log_prefix='rtk-navigation')
        self.control_call = operation, key

    def _poll_control(self):
        if self.control is None:
            return None
        code = self.control.poll()
        if code is None:
            return None
        operation, key = self.control_call
        lines = self.control.log.read_text(errors='replace').splitlines()
        self.control = None
        if code != 0:
            return 'Mux {} failed: {}'.format(operation, lines[-1] if lines else code)
        if operation == 'hold':
            try:
                ack = json.loads(lines[-1])
                if not ack['manual_hold'] or hold_key(ack) != (key[0], key[1] + 1):
                    raise ValueError('unexpected hold acknowledgement')
                self.owned_hold = hold_key(ack)
            except (IndexError, KeyError, TypeError, ValueError) as error:
                return 'Mux hold acknowledgement invalid: ' + str(error)
        else:
            self.owned_hold = None
        return ''

    def _startup(self, release, strict_global=False):
        mux = self._sample('mux')
        if self.startup_pending and self._idle() and mux and self._unique_chain():
            if mux['manual_hold'] and mux['takeover_generation'] == 0 and mux.get('hold_reason') == 'startup':
                self.owned_hold = hold_key(mux)
            self.startup_pending = False
        if release and self.owned_hold is not None and self.control is None and mux:
            if (self._idle() and self._quiet() and self._unique_chain()
                    and mux['manual_hold'] and hold_key(mux) == self.owned_hold
                    and not self._permission(allow_owned_hold=True)
                    and (self.mode != 'map' or not self._global_problem(strict_global))):
                self._begin_control('release', self.owned_hold)

    def _switch(self, mode, preparation=None):
        self.transition = dict(stage='hold', mode=mode, preparation=preparation,
                               deadline=time.monotonic() + self.switch_timeout)
        self.phase, self.reason = 'SWITCHING_NAVIGATION', 'holding old navigation before switching to ' + mode

    def _advance_switch(self, release):
        transition = self.transition
        if self.emergency:
            if transition['stage'] == 'start' and self.nav:
                self.nav.stop()
            raise RuntimeError('emergency stop during navigation switch')
        if time.monotonic() >= transition['deadline']:
            raise RuntimeError('navigation switch timed out at ' + transition['stage'])
        stage = transition['stage']
        if stage == 'hold':
            mux = self._sample('mux')
            if self.control is not None or mux is None or not self._idle() or not self._quiet():
                return
            problem = self._permission(allow_owned_hold=True)
            if problem:
                raise RuntimeError(problem)
            if mux['manual_hold'] and hold_key(mux) == self.owned_hold:
                self.gap_takeover = False
                transition['stage'] = 'exit'
                self.nav.stop()
            elif self._unique_chain():
                self._begin_control('hold', hold_key(mux))
        elif stage == 'exit':
            if self.nav.poll() is None or self._nav_present():
                return
            self._start(transition['mode'], transition['preparation'])
            transition['stage'] = 'start'
        else:
            self._startup(release, strict_global=self.mode == 'map')
            mux = self._sample('mux')
            if not self.startup_pending and mux is not None and self.control is None:
                if release and self._permission():
                    if hold_key(mux) != self.owned_hold or self._permission(allow_owned_hold=True):
                        raise RuntimeError(self._permission())
                    return
                self.transition = None

    def _launch_job(self, name, fn):
        if self.worker is not None:
            return
        job = dict(name=name, deadline=time.monotonic() + 5., done=False)
        self.worker = job

        def run():
            try:
                job['value'] = fn()
            except Exception as error:
                job['error'] = str(error)
            finally:
                job['done'] = True

        threading.Thread(target=run, name=name, daemon=True).start()

    def _job(self, name, fn):
        self._launch_job(name, fn)
        job = self.worker
        if job['name'] != name:
            if not job['done'] and time.monotonic() >= job['deadline']:
                raise RuntimeError(job['name'] + ' timed out before ' + name)
            if job['done']:
                self.worker = None
            return None
        if not job['done']:
            if time.monotonic() >= job['deadline']:
                raise RuntimeError(name + ' timed out')
            return None
        self.worker = None
        if 'error' in job:
            raise RuntimeError(job['error'])
        return job['value']

    def _trigger(self, name):
        service = '/geographic/' + name
        rospy.wait_for_service(service, timeout=2.)
        result = rospy.ServiceProxy(service, Trigger)()
        try:
            details = json.loads(result.message)
        except (ValueError, TypeError):
            details = {'reason': result.message}
        return result.success, details

    def _route(self, remaining, grid, start):
        service = ACTION + '/make_plan'
        rospy.wait_for_service(service, timeout=1.)
        plan = rospy.ServiceProxy(service, self.geo.GetPlan)
        start_pose = self.geo.pose(start, start[2], 'odom')
        for candidate in candidate_goals(start, self.distances):
            if math.hypot(candidate[0] - start[0], candidate[1] - start[1]) > remaining:
                continue
            goal = self.geo.pose(candidate, candidate[2], 'odom')
            if not observed_cell(grid, *candidate[:2]):
                continue
            response = plan(self.geo.GetPlanRequest(start=start_pose, goal=goal, tolerance=0.))
            poses = response.plan.poses
            if len(poses) < 2 or any(p.header.frame_id not in ('', 'odom') for p in poses):
                continue
            points = [(p.pose.position.x, p.pose.position.y) for p in poses]
            if math.dist(points[-1], candidate[:2]) > .15:
                continue
            length = observed_path(grid, [start[:2], *points, candidate[:2]], remaining)
            if length is not None:
                return goal, points
        return ()

    def _finish(self, phase, reason):
        request, self.request = self.request, None
        result = PrepareGeographicResult(phase, reason)
        self.phase, self.reason = ('BLOCKED' if phase == 'FAILED' else 'IDLE'), reason
        if phase == 'READY':
            request.handle.set_succeeded(result)
        elif phase == 'STOPPED':
            request.handle.set_canceled(result)
        else:
            self.last_failure = request
            request.handle.set_aborted(result)

    def _stop_request(self, request, outcome, reason):
        if request.moving:
            self.geo.client.cancel_goal()
            request.must_hold = outcome != 'FIT'
        if outcome != 'FIT':
            self.release_idle = False
            request.must_hold = request.must_hold or self.transition is not None
        if request.stopped_since is None:
            request.stopped_since = time.monotonic()
        request.outcome, request.outcome_reason = outcome, reason
        request.phase, request.reason = 'STOPPING', reason

    def _unconfirmed_hold(self, request, reason):
        if self.nav:
            self.nav.stop()
        self.release_idle = False
        request.outcome, request.outcome_reason = 'FAILED', reason
        request.must_hold = False

    def _settle(self, request, now):
        if self.transition:
            if self.transition['stage'] == 'hold' and request.canceled:
                if self.control is not None:
                    return
                self.transition = None
            else:
                self._advance_switch(release=False)
                if self.transition:
                    return
        if not self._quiet(request.stopped_since) or (request.moving and not self._idle()):
            if now - request.stopped_since <= 8.:
                return
            if request.moving and self.nav:
                self.nav.stop()
                self.release_idle = False
            request.outcome = 'FAILED'
            request.outcome_reason = 'could not verify preparation stop: ' + request.outcome_reason
            request.moving, request.must_hold = False, False
        if request.must_hold:
            mux = self._sample('mux')
            if self._idle() and mux and mux['manual_hold'] and self.control is None:
                request.must_hold = False
            elif now - request.stopped_since > 8.:
                self._unconfirmed_hold(request, 'Mux hold confirmation timed out')
            else:
                if self._idle() and mux and self.control is None and self._unique_chain():
                    self._begin_control('hold', hold_key(mux))
                return
        if request.registration_changed and request.outcome != 'FIT':
            suffix = 'suspend' if request.outcome == 'STOPPED' else 'abort'
            try:
                result = self._job(suffix, lambda: self._trigger('auto_' + suffix + '_registration'))
            except (RuntimeError, rospy.ROSException) as error:
                self._finish('FAILED', 'registration cleanup unconfirmed: ' + str(error))
                return
            if result is None:
                return
            request.registration_changed = False
            if not result[0]:
                self._finish('FAILED', result[1].get('reason', 'registration cleanup rejected'))
                return
        request.moving = False
        if request.outcome != 'FIT':
            self._finish(request.outcome, request.outcome_reason)
            return
        request.phase, request.reason = 'CONFIRMING_REGISTRATION', 'confirming alignment while stopped'
        result = self._job('finish', lambda: self._trigger('auto_finish_registration'))
        if result is None:
            return
        if not result[0]:
            reason = result[1].get('reason', 'alignment commit rejected')
            if reason == 'waiting_for_qualified_RTK_in_stopped_window' and now - request.stopped_since < 8.:
                request.reason = reason
                return
            self._stop_request(request, 'FAILED', reason)
            return
        request.registration_changed = False
        request.require_fresh_global = True
        request.outcome, request.stopped_since = None, None
        request.phase = 'WAITING_GLOBAL'

    def _tick_request(self, request, now):
        if request.canceled and not request.cancellation_started:
            request.cancellation_started = True
            self._stop_request(request, 'STOPPED', 'preparation canceled')
        if request.outcome:
            self._settle(request, now)
            return
        if now - request.started > self.max_duration:
            self._stop_request(request, 'FAILED', 'preparation time budget exhausted')
            return
        if self.transition:
            request.phase, request.reason = 'SWITCHING_NAVIGATION', self.reason
            self._advance_switch(release=True)
            return

        status = self._status()
        if request.generation is not None and status and status.get('generation') != request.generation:
            if (not request.moving and now < request.generation_echo_until
                    and type(status.get('generation')) is int and status['generation'] < request.generation):
                request.phase, request.reason = 'WAITING_RTK', 'waiting for the new registration generation'
                return
            self._stop_request(request, 'FAILED', 'local odom session or registration generation changed')
            return
        if status.get('registered'):
            if self.mode != 'map':
                request.require_fresh_global = True
            problem = self._global_problem(request.require_fresh_global, request.generation)
            if problem:
                request.phase, request.reason = 'WAITING_GLOBAL', problem
                return
            if self.mode == 'odom':
                problem = self._permission(allow_owned_hold=True)
                if self._idle() and self._quiet() and not problem:
                    self._switch('map')
                else:
                    request.phase, request.reason = 'WAITING_NAVIGATION', problem or 'waiting for navigation to stop'
                return
            problem = self._permission()
            if problem or not self._idle():
                request.phase, request.reason = 'WAITING_NAVIGATION', problem or 'another Action owns navigation'
                return
            if self._sample('grid', 2.) is None or self._sample('grid', 2.).header.frame_id != 'map':
                request.phase, request.reason = 'WAITING_NAVIGATION', 'waiting for rebuilt map costmap'
                return
            self._finish('READY', 'geographic map navigation is ready')
            return
        if self.mode == 'map':
            if status and self._idle() and self._quiet() and not self._permission(allow_owned_hold=True):
                self._switch('odom', preparation=True)
            else:
                request.phase = 'WAITING_NAVIGATION'
                request.reason = self._permission(allow_owned_hold=True) or 'waiting for local odometry and navigation to stop'
            return
        if request.moving:
            self._observe_motion(request, status, now)
            return
        problem = self._permission()
        if problem:
            request.phase, request.reason = 'WAITING_NAVIGATION', problem
            return
        if not self._idle() or not self._quiet():
            request.phase, request.reason = 'WAITING_NAVIGATION', 'waiting for idle Action and stationary odometry'
            return
        if not self.preparation:
            if status.get('paired_rtk_usable'):
                self._switch('odom', preparation=True)
            else:
                request.phase = 'WAITING_RTK'
                request.reason = status.get('paired_rtk_reason') or 'waiting for a qualified RTK and odometry pair'
                if now - request.started > self.rtk_wait:
                    self._stop_request(request, 'FAILED', request.reason)
            return
        if request.generation is None and status:
            request.registration_changed = True
            result = self._job('begin', lambda: self._trigger('auto_begin_registration'))
            if result is None:
                return
            if not result[0]:
                request.registration_changed = False
                self._stop_request(request, 'FAILED', result[1].get('reason', 'registration begin failed'))
                return
            request.generation = result[1]['generation']
            request.generation_echo_until = now + .75
            return
        if (not status or request.generation is None or status.get('state') != 'collecting'
                or not status.get('paired_rtk_usable')):
            request.phase = 'WAITING_RTK'
            request.reason = status.get('paired_rtk_reason') or 'waiting for qualified paired RTK and odometry'
            if now - request.started > self.rtk_wait:
                self._stop_request(request, 'FAILED', request.reason)
            return
        progress = status.get('alignment_progress') or {}
        if progress.get('ready'):
            self._stop_request(request, 'FIT', 'alignment evidence is ready')
            return
        grid = self._sample('grid', 2.)
        if grid is None or grid.header.frame_id != 'odom':
            request.phase, request.reason = 'WAITING_NAVIGATION', 'waiting for live odom costmap'
            return
        request.phase, request.reason = 'PLANNING_PREPARATION', 'finding an observed, bounded odom route'
        start = self._local_pose()
        margin = self.max_distance - request.distance
        result = self._job('plan', lambda: self._route(margin, grid, start))
        if result is None:
            return
        if not result:
            self._stop_request(request, 'FAILED', 'no observed, traversable odom route within the remaining budget')
            return
        goal, points = result
        # Recheck the live grid and control ownership after the service completes.
        live = self._sample('grid', 2.)
        if (self._permission() or not self._idle() or not self._unique_chain()
                or live is None or live.header.frame_id != 'odom'):
            return
        end = goal.pose.position
        if observed_path(live, [start[:2], *points, (end.x, end.y)], margin) is None:
            return
        self.geo.client.send_goal(self.geo.MoveBaseGoal(target_pose=goal))
        request.moving = True
        request.last_position = start[:2]
        request.progress_at, request.progress_distance = now, request.distance
        request.phase, request.reason = 'PREPARING', 'following bounded odom route and pairing RTK'

    def _observe_motion(self, request, status, now):
        collecting = status.get('state') == 'collecting' and status.get('registration_mode') == 'automatic'
        problem = self._permission()
        if not collecting:
            problem = problem or 'automatic registration collection stopped'
        if not status.get('paired_rtk_usable'):
            problem = problem or status.get('paired_rtk_reason') or 'paired RTK became unavailable'
        if problem:
            self._stop_request(request, 'FAILED', problem)
            return
        position = self._local_pose()[:2]
        request.distance += math.dist(position, request.last_position)
        request.last_position = position
        if request.distance >= request.progress_distance + .08:
            request.progress_at, request.progress_distance = now, request.distance
        if (status.get('alignment_progress') or {}).get('ready'):
            self._stop_request(request, 'FIT', 'alignment evidence is ready')
        elif request.distance >= self.max_distance:
            self._stop_request(request, 'FAILED', 'preparation distance budget exhausted')
        elif now - request.progress_at > self.no_progress:
            self._stop_request(request, 'FAILED', 'preparation route made no progress')
        elif self.geo.client.get_state() == 3:
            request.moving = False
        elif self.geo.client.get_state() in (2, 4, 5, 8, 9):
            self._stop_request(request, 'FAILED', 'temporary odom Action ended without reaching its goal')

    def _publish(self):
        request = self.request
        phase, reason = (request.phase, request.reason) if request else (self.phase, self.reason)
        if request:
            request.handle.publish_feedback(PrepareGeographicFeedback(phase, reason))
        alignment = self._status().get('alignment_progress') or {}
        displayed = request or self.last_failure
        payload = dict(action=ACTION, mode=self.mode, preparation=self.preparation, phase=phase, reason=reason,
                       busy=bool(request or self.transition), stamp=rospy.Time.now().to_sec(),
                       task_id=displayed.handle.get_goal().task_id if displayed else None,
                       target=displayed.target if displayed else None,
                       distance_m=displayed.distance if displayed else None,
                       span_m=alignment.get('baseline_m'), rmse_m=alignment.get('residual_m'))
        self.owner_pub.publish(String(data=json.dumps(payload, allow_nan=False)))

    def tick(self):
        control_error = self._poll_control()
        if control_error:
            raise RuntimeError(control_error)
        if self.nav is None:
            if self.initial_mode == 'map' and self._global_problem(strict=True):
                self.phase, self.reason = 'WAITING_GLOBAL', self._global_problem(strict=True)
                if self.request:
                    self.request.phase, self.request.reason = self.phase, self.reason
                    if self.request.canceled or time.monotonic() - self.request.started > self.rtk_wait:
                        self._stop_request(self.request, 'STOPPED' if self.request.canceled else 'FAILED',
                                           self.reason)
                        self._settle(self.request, time.monotonic())
                return
            self._start(self.initial_mode)
        if not self.transition or self.transition['stage'] != 'exit':
            if self.nav.poll() is not None and not (self.request and self.request.outcome):
                raise RuntimeError('field navigation launch exited; see ' + str(self.nav.log))
        if not self.transition:
            request = self.request
            release = self.release_idle if request is None else not (request.canceled or request.outcome)
            self._startup(release, strict_global=bool(request and request.require_fresh_global))
        if self.request:
            self._tick_request(self.request, time.monotonic())
        elif self.transition:
            self._advance_switch(release=True)
        elif self.phase != 'BLOCKED':
            self.phase = 'IDLE' if self.mode and self._idle() else 'WAITING_NAVIGATION'
            self.reason = self._permission() or self.mode + ' navigation is ready'

    def run(self):
        while not rospy.is_shutdown():
            with self.lock:
                try:
                    self._process_events()
                    self.tick()
                except (RuntimeError, OSError, rospy.ROSException, tf2_ros.TransformException, ValueError) as error:
                    reason = str(error)
                    if self.transition:
                        self.transition = None
                        self.release_idle = False
                    if self.request and not self.request.outcome:
                        self._stop_request(self.request, 'FAILED', reason)
                    elif self.request:
                        if self.request.must_hold:
                            self._unconfirmed_hold(self.request, reason)
                        else:
                            self._stop_request(self.request, 'FAILED', reason)
                    else:
                        self.phase, self.reason = 'BLOCKED', reason
                    rospy.logwarn_throttle(2., '%s', reason)
                self._publish()
            time.sleep(.05)

    def shutdown(self):
        if self.request and self.request.moving:
            self.geo.client.cancel_goal()
        for process in (self.control, self.nav):
            if process:
                process.stop()
        cleanup = None
        if self.request and self.request.registration_changed:
            # The sensors can outlive a headless navigation owner.
            def abort_pending():
                job = self.worker
                deadline = time.monotonic() + 2.
                while job and not job['done'] and time.monotonic() < deadline:
                    time.sleep(.02)
                try:
                    rospy.ServiceProxy('/geographic/auto_abort_registration', Trigger)()
                except (rospy.ROSException, rospy.ServiceException):
                    pass

            cleanup = threading.Thread(target=abort_pending, daemon=True)
            cleanup.start()
        deadline = time.monotonic() + 10.
        while time.monotonic() < deadline:
            active = [process for process in (self.control, self.nav)
                      if process and process.poll() is None]
            if not active and (cleanup is None or not cleanup.is_alive()):
                break
            time.sleep(.05)


def main():
    with acquire_local_owner():
        master = rosgraph.Master('/slicer_navigation_runtime_check')
        try:
            master.lookupNode('/slicer_navigation_runtime')
        except rosgraph.MasterError:
            pass
        else:
            raise RuntimeError('a shared field navigation owner is already registered on this master')
        rospy.init_node('slicer_navigation_runtime')
        runtime = NavigationRuntime()
        try:
            runtime.run()
        finally:
            runtime.shutdown()


if __name__ == '__main__':
    main()
