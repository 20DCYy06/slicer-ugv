"""Single Action owner scheduling the real task core; ROS I/O is injected.

No velocity interface, alternate controller, or feedback ACK dependency.
The old Action remains owned until its terminal acknowledgment is observed.
"""
import math
import time


TERMINAL = (2, 3, 4, 5, 8)  # LOST is not proof that the server stopped.


class TaskExecutor:
    def __init__(self, core, backend, clock=time.monotonic, via_pass_radius_m=.25,
                 segment_timeout_s=120., cancel_timeout_s=3., stop_timeout_s=8.):
        self.core, self.backend, self.clock = core, backend, clock
        values = (via_pass_radius_m, segment_timeout_s, cancel_timeout_s, stop_timeout_s)
        if not all(type(x) in (int, float) and math.isfinite(x) and x > 0 for x in values):
            raise ValueError('executor settings must be finite and positive')
        self.radius, self.timeout, self.cancel_timeout, self.stop_timeout = values
        self.active, self.cancel_since, self.verify_since = None, None, None
        self.stop_since, self.own_hold, self.stop_held = None, None, False
        self.runtime_phase, self.action, self.arrival = 'IDLE', None, None
        self.current_navigation_target, self.last_error = None, ''
        self.stop_result = None
        self.preparing, self.preparation_cancel_requested = False, False
        self.preparation_retry, self.preparation_ready, self.preparation_stopped = False, False, False
        self.preparation_reason = ''

    def recover_orphan(self):
        """A new receiver found an old server goal, not a resumable task."""
        if self.core.snapshot()['task_id'] is not None or self.active is not None:
            raise ValueError('startup cleanup is only for an idle new receiver')
        self.stop_since, self.stop_held, self.stop_result = self.clock(), False, None
        self.runtime_phase = 'STARTUP_STOPPING'

    def command(self, command):
        def validate(final, vias):
            try:
                route_validator(final, vias)
            except Exception as error:
                # A backend-specific route check must leave the original task intact.
                raise ValueError('route unavailable: '+str(error)) from error
        route_validator = getattr(self.backend, 'validate_route', None)
        result = self.core.handle(command, validate_route=validate if route_validator else None)
        if result['accepted'] and not result.get('duplicate'):
            self.last_error = ''
            self.arrival = None
            if self.core.snapshot()['phase'] in ('PAUSING', 'CANCELING'):
                self.stop_since, self.stop_held = self.clock(), False
                self.stop_result = None
        return result

    @staticmethod
    def hold_key(state):
        return state.get('mux_instance_id'), state.get('takeover_generation')

    def _hold(self):
        if self.stop_held:
            return
        before = self.backend.mux_state()
        # Do not claim an external/manual hold as ours merely by taking it again.
        if before['manual_hold'] and self.hold_key(before) != self.own_hold:
            self.own_hold = None
        elif not before['manual_hold']:
            after = self.backend.hold()
            if not after['manual_hold']:
                raise ValueError('Mux hold not confirmed')
            self.own_hold = self.hold_key(after)
        self.stop_held = True

    def _block(self, reason):
        if self.core.snapshot()['phase'] != 'BLOCKED' or self.last_error != reason:
            self.core.block(reason)
        self.last_error, self.runtime_phase = reason, 'BLOCKED'
        if self.stop_since is None:
            self.stop_since, self.stop_held = self.clock(), False

    def _cancel(self):
        if self.cancel_since is None:
            self.cancel_since = self.clock()
            self.backend.cancel()
        self.runtime_phase = 'SWITCHING'

    def _clear_active(self):
        self.active, self.cancel_since, self.verify_since = None, None, None

    def _stopping(self):
        if self.preparation_retry:
            busy = self.backend.active_server_goal()
            if busy is not False:
                # The runtime can confirm stop after its old navigation process
                # has exited. There may be no Action/Mux to hold in that gap.
                if self.preparation_stopped and busy is None:
                    self._mark_stopped(dict(confirmed=True, source='geographic_preparation'))
                elif self.clock()-self.stop_since > self.stop_timeout:
                    self._block('stop not confirmed: fresh idle navigation unavailable')
                return
        self._hold()
        if self.active is not None:
            return
        quiet = self.backend.stopped(self.stop_since)
        self.stop_result = quiet
        if quiet.get('confirmed'):
            self._mark_stopped(quiet)
        elif self.clock()-self.stop_since > self.stop_timeout:
            self._block('stop not confirmed: '+quiet.get('reason', 'quiet window unavailable'))

    def _mark_stopped(self, quiet):
        self.stop_result = quiet
        self.core.mark_stopped()
        self.stop_since, self.preparation_stopped = None, False
        self.preparation_reason = ''
        self.runtime_phase = self.core.snapshot()['phase']

    def _preparation_failed(self, reason):
        self.preparing = False
        self.preparation_reason = reason or 'geographic preparation failed'
        # The runtime owns the temporary Action and its stopping path. A failed
        # admission can also mean an unrelated navigator was already active.
        self.core.block(self.preparation_reason)
        self.last_error, self.runtime_phase = self.preparation_reason, 'BLOCKED'

    def _begin_preparation(self, state):
        self.preparation_retry, self.preparation_ready = True, False
        self.current_navigation_target, self.action = None, None
        self.preparation_cancel_requested, self.preparation_stopped = False, False
        try:
            if self.own_hold is not None:
                mux = self.backend.mux_state()
                if self.hold_key(mux) == self.own_hold and mux['manual_hold']:
                    if mux['emergency_stop_latched'] or mux['teleop_fresh']:
                        raise ValueError('existing emergency stop/teleop remains')
                    self._release_own_hold()
                self.own_hold, self.stop_held = None, False
            final = state.get('final_goal') or state.get('target')
            self.backend.prepare_start(state['task_id'], final)
        except Exception as error:
            self._preparation_failed(str(error))
            return
        self.preparing = True
        self.runtime_phase, self.preparation_reason = 'WAITING_PREPARATION', ''

    def _advance_preparation(self):
        stopping = self.stop_since is not None
        try:
            if stopping and not self.preparation_cancel_requested:
                self.backend.prepare_cancel()
                self.preparation_cancel_requested = True
            phase, reason = self.backend.prepare_poll()
        except Exception as error:
            self._preparation_failed(str(error))
            self.backend.prepare_cancel()
            return
        self.runtime_phase = self.core.snapshot()['phase'] if stopping else phase
        self.preparation_reason = reason
        if phase not in ('READY', 'STOPPED', 'FAILED'):
            return
        self.preparing = False
        if stopping:
            self.preparation_stopped = phase == 'STOPPED'
        elif phase == 'READY':
            self.preparation_retry, self.preparation_ready = False, True
            self.preparation_reason = ''
        else:
            self._preparation_failed(reason)

    def _release_own_hold(self):
        if self.backend.release(self.own_hold)['manual_hold']:
            raise ValueError('Mux release not confirmed')

    def tick(self):
        try:
            self._tick()
        except Exception as error:
            self._block(str(error))
            # A failed release/conversion/TF must not leave a goal running.
            try:
                if self.active is not None:
                    self._cancel()
            except Exception as stop_error:
                self.last_error += '; stop unconfirmed: '+str(stop_error)
        finally:
            # Mux hold is independent of Action transport and its terminal ACK.
            if not self.preparation_retry and (self.stop_since is not None or
                    self.core.snapshot()['phase'] in ('PAUSING', 'CANCELING', 'BLOCKED')):
                try:
                    self._hold()
                except Exception as error:
                    self.last_error += '; Mux hold unconfirmed: '+str(error)

    def _tick(self):
        if self.preparing:
            self._advance_preparation()
            return
        state = self.core.snapshot()
        now = self.clock()
        if self.active is not None:
            segment, target, began = self.active
            valid = state.get('active') and state['active']['token'] == segment['token']
            if not valid:
                self._cancel()
            else:
                problem = self.backend.health(target)
                mux = self.backend.mux_state()
                if problem or mux['manual_hold'] or mux['emergency_stop_latched'] or mux['teleop_fresh']:
                    self._block(problem or 'external Mux hold/emergency stop/teleop')
                    self._cancel()
                elif now-began > self.timeout and self.verify_since is None:
                    self._block('navigation segment timeout')
                    self._cancel()
            self.action = self.backend.poll()
            if self.cancel_since is not None:
                if self.action['state'] in TERMINAL:
                    self._clear_active()
                    if self.stop_since is not None:
                        self._stopping()
                elif now-self.cancel_since > self.cancel_timeout:
                    self._block('Action cancellation not acknowledged; no replacement goal sent')
                    self._hold()
                return
            if self.action['state'] == 9:
                self._block('Action LOST; server termination is unconfirmed')
                self._cancel()
                self._hold()
                return
            if segment['kind'] == 'via':
                x, y = self.backend.position()
                distance = math.hypot(x-target['map_xyz'][0], y-target['map_xyz'][1])
                if distance <= self.radius:
                    self.core.settle(segment['token'], True, result=dict(position_passed=True,
                                     distance_m=distance, radius_m=self.radius))
                    if self.action['state'] in TERMINAL:
                        self._clear_active()
                    else:
                        self._cancel()
                elif self.action['state'] in TERMINAL:
                    self.core.settle(segment['token'], False, 'via not reached: '+str(self.action))
                    self._clear_active()
                    self._block('via not reached; remaining route retained')
                return
            if self.action['state'] in TERMINAL:
                if self.action['state'] != 3:
                    self.core.settle(segment['token'], False, 'final Action failed: '+str(self.action))
                    self._clear_active()
                    self._block('final Action failed; target retained')
                    return
                if self.verify_since is None:
                    self.verify_since, self.runtime_phase = now, 'VERIFYING'
                    return
                self.arrival = self.backend.review(target, self.action, self.verify_since)
                stop = self.arrival.get('stop', {})
                geo = self.arrival.get('geographic', {})
                if geo.get('qualified') and stop.get('confirmed'):
                    success = geo.get('confirmed') is True
                    self._hold()
                    self.core.settle(segment['token'], success,
                                     '' if success else geo.get('reason', 'geographic arrival unconfirmed'),
                                     result=self.arrival)
                    self._clear_active()
                    self.runtime_phase = self.core.snapshot()['phase']
                elif now-self.verify_since > self.stop_timeout:
                    self.core.settle(segment['token'], False, 'parking/geographic review unconfirmed',
                                     result=self.arrival)
                    self._clear_active()
                    self._block('parking/geographic review unconfirmed')
                return
        state = self.core.snapshot()
        if (self.stop_since is not None or state['phase'] in ('PAUSING', 'CANCELING')
                or (state['phase'] == 'BLOCKED' and not self.preparation_retry)):
            if self.stop_since is None:
                self.stop_since = now
            self._stopping()
            return
        if state['phase'] != 'READY' or state['paused']:
            self.runtime_phase = state['phase']
            return
        if not self.preparation_ready and (self.preparation_retry or self.backend.needs_preparation()):
            self._begin_preparation(state)
            return
        busy = self.backend.active_server_goal()
        if busy is not False:
            self.runtime_phase = 'WAITING_NAVIGATION' if busy is None else 'NAVIGATION_BUSY'
            return
        self.preparation_ready = False
        problem = self.backend.health()
        if problem:
            raise ValueError(problem)
        mux = self.backend.mux_state()
        if mux['emergency_stop_latched'] or mux['teleop_fresh']:
            raise ValueError('existing emergency stop/teleop remains')
        if mux['manual_hold']:
            if self.hold_key(mux) != self.own_hold:
                raise ValueError('external/startup hold; existing readiness release required')
            self._release_own_hold()
        self.own_hold, self.stop_held, self.stop_since = None, False, None
        segment = self.core.activate()
        if segment is not None:
            target = self.backend.target(segment['point'])
            target.update(task_id=segment['task_id'], route_revision=segment['revision'],
                          segment_token=segment['token'], segment_kind=segment['kind'])
            self.current_navigation_target = target
            self.active = segment, target, now
            self.backend.start(target)
            self.action, self.arrival, self.runtime_phase = None, None, 'NAVIGATING'

    def snapshot(self):
        return dict(self.core.snapshot(), runtime_phase=self.runtime_phase,
                    current_navigation_target=self.current_navigation_target,
                    action=self.action, arrival=self.arrival, last_error=self.last_error,
                    stop=self.stop_result,
                    preparation_reason=self.preparation_reason,
                    cancellation_pending=(self.cancel_since is not None or
                                          self.preparing and self.preparation_cancel_requested))

    def shutdown(self):
        if self.preparing:
            self.backend.prepare_cancel()
            return
        if self.active is None and self.own_hold is None and self.stop_since is None:
            return  # Starting/stopping an unused interface does not take control.
        try:
            if self.active is not None:
                self.backend.cancel()
        finally:
            self.stop_held = False
            self._hold()
