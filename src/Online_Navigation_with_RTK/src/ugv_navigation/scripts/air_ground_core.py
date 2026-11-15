#!/usr/bin/env python3
"""Pure WGS84 task state, independent of ROS and navigation execution."""
import copy
import json
import math
from collections import OrderedDict


OPERATIONS = frozenset(('SET_TASK', 'REPLACE_VIA_POINTS', 'PAUSE', 'RESUME', 'CANCEL'))
COMMAND_FIELDS = frozenset(('protocol_version', 'server_id', 'sequence', 'request_id',
                            'task_id', 'operation', 'final_goal', 'via_points'))
POINT_FIELDS = frozenset(('latitude', 'longitude', 'altitude_ellipsoid_m', 'yaw'))


def _finite_number(value):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return False
    try:
        return math.isfinite(value)
    except OverflowError:
        return False


def _point(value, via=False):
    if not isinstance(value, dict) or set(value) - POINT_FIELDS:
        raise ValueError('point must contain only WGS84 point fields')
    latitude, longitude = value.get('latitude'), value.get('longitude')
    if not _finite_number(latitude) or not -90. <= latitude <= 90.:
        raise ValueError('latitude must be finite degrees in [-90, 90]')
    if not _finite_number(longitude) or not -180. <= longitude <= 180.:
        raise ValueError('longitude must be finite degrees in [-180, 180]')
    altitude, yaw = value.get('altitude_ellipsoid_m'), value.get('yaw')
    if altitude is not None and not _finite_number(altitude):
        raise ValueError('altitude_ellipsoid_m must be finite or unspecified')
    if yaw is not None and not _finite_number(yaw):
        raise ValueError('yaw must be finite radians or unspecified')
    if via and yaw is not None:
        raise ValueError('via points have position-only semantics; yaw must be unspecified')
    return dict(latitude=latitude, longitude=longitude, altitude_ellipsoid_m=altitude, yaw=yaw)


class TaskCore:
    """Keep task intent; the caller owns Action cancellation and stopping.

    Sequence numbers are positive uint64 and global to this server boot.
    ``last_sequence`` is None until the first valid envelope. Any strictly
    newer, valid envelope consumes its sequence, even when its batch, task ID,
    or task state is rejected. Bad envelopes, old sequences, and request-ID
    content conflicts do not advance it. Last 256 receipts are replayable;
    the sequence watermark still rejects older commands after eviction.

    ``activate`` may be called only after the runner has observed termination
    of its previous Action. A route update invalidates intent immediately; it
    does not assert that the physical runner has already stopped.
    """

    def __init__(self, server_id):
        self.server_id = server_id
        self.task_id = None
        self.revision = 0
        self.paused = False
        self.phase = 'IDLE'
        self.final_goal = None
        self.remaining_via_points = []
        self.active = None
        self.last_command = None
        self.last_task_result = None
        self.last_sequence = None
        self.reason = ''
        self._receipts = OrderedDict()
        self._ended_task_ids = set()
        self._token_counter = 0

    def snapshot(self):
        """Return an independent JSON-compatible state snapshot."""
        return copy.deepcopy(dict(protocol_version=1, server_id=self.server_id,
                                  task_id=self.task_id, revision=self.revision,
                                  paused=self.paused, phase=self.phase,
                                  final_goal=self.final_goal,
                                  remaining_via_points=self.remaining_via_points,
                                  active=self.active, last_command=self.last_command,
                                  last_task_result=self.last_task_result,
                                  last_sequence=self.last_sequence, reason=self.reason))

    def activate(self):
        """Claim the next segment only after previous execution has stopped."""
        if self.paused or self.phase != 'READY' or self.task_id is None:
            return None
        self._token_counter += 1
        via = bool(self.remaining_via_points)
        self.active = dict(token='{}:{}'.format(self.server_id, self._token_counter),
                           kind='via' if via else 'final',
                           point=copy.deepcopy(self.remaining_via_points[0] if via else self.final_goal),
                           task_id=self.task_id, revision=self.revision)
        self.phase = 'EXECUTING'
        return copy.deepcopy(self.active)

    def handle(self, command, validate_route=None):
        """Validate a complete command, then atomically update task intent.

        ``validate_route(final_goal, via_points)`` may reject a SET, route
        replacement, or RESUME with ValueError before task mutation. It sees
        independent copies of the complete candidate route. Duplicate receipts
        bypass validation; PAUSE and CANCEL never depend on route conversion.
        """
        fingerprint = None
        identifiable = (isinstance(command, dict) and command.get('server_id') == self.server_id and
                        isinstance(command.get('request_id'), str) and command['request_id'].strip())
        if identifiable:
            try:
                # NaN is fingerprintable for stable rejection replay, never a
                # valid coordinate or a number copied into published snapshots.
                fingerprint = json.dumps(command, sort_keys=True, separators=(',', ':'), allow_nan=True)
            except (TypeError, ValueError):
                pass
            if fingerprint is not None:
                previous = self._receipts.get(command['request_id'])
                if previous is not None:
                    if previous[0] != fingerprint:
                        return dict(accepted=False, reason='request_id was already used with different content',
                                    duplicate=False)
                    return dict(copy.deepcopy(previous[1]), duplicate=True)

        problem = self._envelope_problem(command)
        if problem:
            if fingerprint is not None:
                return self._record(command, fingerprint, False, problem)
            return dict(accepted=False, reason=problem, duplicate=False)
        if fingerprint is None:
            return dict(accepted=False, reason='command must be a JSON-compatible object', duplicate=False)

        if self.last_sequence is not None and command['sequence'] <= self.last_sequence:
            return self._record(command, fingerprint, False, 'sequence is not newer than last_sequence')
        self.last_sequence = command['sequence']
        try:
            final, vias = self._route_payload(command)
        except ValueError as error:
            return self._record(command, fingerprint, False, str(error))

        operation, task_id = command['operation'], command['task_id']
        if operation == 'SET_TASK':
            if task_id == self.task_id or task_id in self._ended_task_ids:
                return self._record(command, fingerprint, False, 'SET_TASK requires a new task_id for this boot')
        elif (task_id != self.task_id or self.task_id is None or
              self.phase in ('IDLE', 'COMPLETED', 'CANCELED', 'CANCELING')):
            return self._record(command, fingerprint, False, 'task_id does not name an unfinished task')

        if validate_route is not None and operation in ('SET_TASK', 'REPLACE_VIA_POINTS', 'RESUME'):
            candidate_final = final if operation == 'SET_TASK' else self.final_goal
            candidate_vias = self.remaining_via_points if operation == 'RESUME' else vias
            try:
                validate_route(copy.deepcopy(candidate_final), copy.deepcopy(candidate_vias))
            except ValueError as error:
                return self._record(command, fingerprint, False, str(error))

        route_phase = 'READY'
        if self.paused:
            route_phase = 'PAUSING' if self.phase == 'PAUSING' else 'PAUSED'
        if operation == 'SET_TASK':
            if self.task_id is not None and self.task_id not in self._ended_task_ids:
                self._finish('REPLACED', 'replaced by task {}'.format(task_id))
            self.task_id, self.final_goal, self.remaining_via_points = task_id, final, vias
            self._invalidate()
            self.phase = route_phase
            self.reason = ''
        elif operation == 'REPLACE_VIA_POINTS':
            self.remaining_via_points = vias
            self._invalidate()
            self.phase = route_phase
            self.reason = ''
        elif operation == 'PAUSE':
            self._invalidate()
            self.paused, self.phase, self.reason = True, 'PAUSING', ''
        elif operation == 'RESUME':
            self._invalidate()
            self.paused, self.phase, self.reason = False, 'READY', ''
        elif operation == 'CANCEL':
            self._invalidate()
            self.phase, self.reason = 'CANCELING', ''
        return self._record(command, fingerprint, True, '')

    def settle(self, token, success, reason='', result=None):
        """Settle only the current token, after navigation/arrival checks."""
        if (self.active is None or self.phase != 'EXECUTING' or self.active['token'] != token or
                self.active['task_id'] != self.task_id or self.active['revision'] != self.revision):
            return False
        kind = self.active['kind']
        self.active = None
        if not success:
            self.block(reason or 'navigation failed')
        elif kind == 'via':
            self.remaining_via_points.pop(0)
            self.phase, self.reason = 'READY', ''
        else:
            self.phase, self.reason = 'COMPLETED', ''
            self._finish('COMPLETED', reason, result)
        return True

    def mark_stopped(self):
        """Acknowledge runner termination/stop; never advance a route."""
        if self.phase == 'PAUSING':
            self.phase = 'PAUSED'
        elif self.phase == 'CANCELING':
            self.active = None
            self.remaining_via_points = []
            self.phase = 'CANCELED'
            self._finish('CANCELED', 'canceled by command')

    def block(self, reason):
        """Preserve the unfinished route for an explicit retry or reroute."""
        if self.task_id is None or self.phase in ('IDLE', 'COMPLETED', 'CANCELED', 'CANCELING'):
            return
        self._invalidate()
        self.paused, self.phase, self.reason = True, 'BLOCKED', str(reason)

    def _envelope_problem(self, command):
        if not isinstance(command, dict):
            return 'command must be an object'
        if set(command) - COMMAND_FIELDS:
            return 'unknown command field'
        if type(command.get('protocol_version')) is not int or command['protocol_version'] != 1:
            return 'unsupported protocol_version'
        if command.get('server_id') != self.server_id:
            return 'server_id does not match this receiver boot'
        sequence = command.get('sequence')
        if type(sequence) is not int or not 1 <= sequence < 2**64:
            return 'sequence must be positive uint64'
        for key in ('request_id', 'task_id'):
            if not isinstance(command.get(key), str) or not command[key].strip():
                return key + ' must be a nonempty string'
        operation = command.get('operation')
        if not isinstance(operation, str) or operation not in OPERATIONS:
            return 'unsupported operation'
        return ''

    def _route_payload(self, command):
        operation = command['operation']
        if operation == 'SET_TASK':
            final = _point(command.get('final_goal'))
        else:
            if command.get('final_goal') is not None:
                raise ValueError('final_goal is only valid for SET_TASK')
            final = None
        vias = command.get('via_points', [])
        if operation == 'REPLACE_VIA_POINTS' and 'via_points' not in command:
            raise ValueError('REPLACE_VIA_POINTS requires via_points, including an explicit empty list')
        if not isinstance(vias, list):
            raise ValueError('via_points must be a list')
        if operation not in ('SET_TASK', 'REPLACE_VIA_POINTS') and vias:
            raise ValueError('via_points are only valid for route commands')
        return final, [_point(value, via=True) for value in vias]

    def _invalidate(self):
        self.revision += 1
        self.active = None

    def _finish(self, status, reason='', result=None):
        self._ended_task_ids.add(self.task_id)
        self.last_task_result = dict(task_id=self.task_id, status=status, reason=reason,
                                     result=copy.deepcopy(result))

    def _record(self, command, fingerprint, accepted, reason):
        receipt = dict(accepted=accepted, reason=reason, duplicate=False,
                       request_id=command.get('request_id'),
                       sequence=command.get('sequence') if type(command.get('sequence')) is int else None,
                       task_id=command.get('task_id') if isinstance(command.get('task_id'), str) else None,
                       operation=command.get('operation') if isinstance(command.get('operation'), str) else None)
        self.last_command = copy.deepcopy(receipt)
        self._receipts[command['request_id']] = (fingerprint, copy.deepcopy(receipt))
        if len(self._receipts) > 256:
            self._receipts.popitem(last=False)
        return receipt
