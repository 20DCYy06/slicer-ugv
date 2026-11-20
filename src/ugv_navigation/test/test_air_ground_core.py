#!/usr/bin/env python3
"""Task protocol/state boundaries; no ROS master, network, or hardware."""
import copy
import json
import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
try:
    from air_ground_core import TaskCore
except ImportError:
    TaskCore = None


def point(latitude=30., longitude=120., altitude=None, yaw=None):
    return dict(latitude=latitude, longitude=longitude,
                altitude_ellipsoid_m=altitude, yaw=yaw)


class TaskCoreTests(unittest.TestCase):
    def setUp(self):
        self.assertIsNotNone(TaskCore, 'pure air-ground task core must be implemented')
        self.core = TaskCore('boot-1')
        self.sequence = 0

    def command(self, operation='SET_TASK', task_id='task-1', **fields):
        self.sequence += 1
        result = dict(protocol_version=1, server_id='boot-1', sequence=self.sequence,
                      request_id='request-{}'.format(self.sequence), task_id=task_id,
                      operation=operation)
        if operation == 'SET_TASK':
            result.update(final_goal=point(), via_points=[])
        result.update(fields)
        return result

    def accept(self, operation='SET_TASK', task_id='task-1', **fields):
        result = self.core.handle(self.command(operation, task_id, **fields))
        self.assertTrue(result['accepted'], result)
        self.assertFalse(result['duplicate'])
        return result

    def validate(self, command, callback):
        try:
            return self.core.handle(command, validate_route=callback)
        except TypeError as error:
            self.fail('handle must support an atomic route validation callback: {}'.format(error))

    def test_boot_is_idle_and_cannot_start_motion_without_a_task(self):
        state = self.core.snapshot()
        self.assertEqual((state['protocol_version'], state['server_id']), (1, 'boot-1'))
        self.assertEqual((state['phase'], state['task_id'], state['revision']), ('IDLE', None, 0))
        self.assertIsNone(state['last_sequence'])
        self.assertFalse(state['paused'])
        self.assertIsNone(self.core.activate())
        json.dumps(state, allow_nan=False)

    def test_single_final_target_needs_explicit_successful_settlement(self):
        self.accept(final_goal=point(31., 121., 16., -.5))
        self.assertEqual(self.core.snapshot()['phase'], 'READY')
        segment = self.core.activate()
        self.assertEqual((segment['kind'], segment['task_id'], segment['point']),
                         ('final', 'task-1', point(31., 121., 16., -.5)))
        self.assertEqual(segment['revision'], self.core.snapshot()['revision'])
        self.assertIsNone(self.core.activate(), 'a second claim cannot duplicate execution')
        self.assertEqual(self.core.snapshot()['phase'], 'EXECUTING')
        self.assertIsNone(self.core.snapshot()['last_task_result'])
        review = {'stop': {'confirmed': True}, 'geographic': {'confirmed': True}}
        self.assertTrue(self.core.settle(segment['token'], True, result=review))
        state = self.core.snapshot()
        self.assertEqual(state['phase'], 'COMPLETED')
        self.assertEqual(state['last_task_result'], dict(task_id='task-1', status='COMPLETED',
                                                       reason='', result=review))
        self.assertIsNone(self.core.activate())

    def test_via_success_advances_in_order_without_completing_final_goal(self):
        self.accept(via_points=[point(30.1), point(30.2)])
        first = self.core.activate()
        self.assertEqual(first['kind'], 'via')
        self.assertEqual(len(self.core.snapshot()['remaining_via_points']), 2)
        self.assertTrue(self.core.settle(first['token'], True))
        self.assertEqual(self.core.snapshot()['remaining_via_points'], [point(30.2)])
        self.assertEqual(self.core.snapshot()['phase'], 'READY')
        self.assertIsNone(self.core.snapshot()['last_task_result'])
        second = self.core.activate()
        self.assertEqual(second['point'], point(30.2))
        self.assertNotEqual(second['token'], first['token'])
        self.core.settle(second['token'], True)
        self.assertEqual(self.core.activate()['kind'], 'final')

    def test_replace_via_points_includes_current_and_preserves_final(self):
        self.accept(via_points=[point(30.1), point(30.2)])
        old = self.core.activate()
        self.accept('REPLACE_VIA_POINTS', via_points=[point(30.3), point(30.4)])
        state = self.core.snapshot()
        self.assertEqual(state['final_goal'], point())
        self.assertEqual(state['remaining_via_points'], [point(30.3), point(30.4)])
        self.assertGreater(state['revision'], old['revision'])
        self.assertIsNone(state['active'])
        self.assertFalse(self.core.settle(old['token'], True))
        self.assertEqual(self.core.activate()['point'], point(30.3))

    def test_empty_replacement_goes_directly_to_retained_final(self):
        self.accept(via_points=[point(30.1)])
        self.core.activate()
        self.accept('REPLACE_VIA_POINTS', via_points=[])
        segment = self.core.activate()
        self.assertEqual((segment['kind'], segment['point']), ('final', point()))

    def test_set_task_ends_previous_task_and_stale_result_cannot_finish_new_task(self):
        self.accept(via_points=[point(30.1)])
        old = self.core.activate()
        self.accept(task_id='task-2', final_goal=point(32.))
        state = self.core.snapshot()
        self.assertEqual((state['task_id'], state['phase']), ('task-2', 'READY'))
        self.assertEqual(state['remaining_via_points'], [])
        self.assertEqual(state['last_task_result']['task_id'], 'task-1')
        self.assertEqual(state['last_task_result']['status'], 'REPLACED')
        new = self.core.activate()
        self.assertFalse(self.core.settle(old['token'], False, 'old failure'))
        self.assertEqual(self.core.snapshot()['active'], new)
        self.assertFalse(self.core.settle(old['token'], True))
        self.assertEqual(self.core.snapshot()['phase'], 'EXECUTING')

    def test_pause_preserves_current_via_until_stopped_and_resume_retries_it(self):
        self.accept(via_points=[point(30.1), point(30.2)])
        old = self.core.activate()
        self.accept('PAUSE')
        state = self.core.snapshot()
        self.assertTrue(state['paused'])
        self.assertEqual(state['phase'], 'PAUSING')
        self.assertEqual(state['remaining_via_points'], [point(30.1), point(30.2)])
        self.assertFalse(self.core.settle(old['token'], True))
        self.assertIsNone(self.core.activate())
        self.core.mark_stopped()
        self.assertEqual(self.core.snapshot()['phase'], 'PAUSED')
        self.accept('RESUME')
        self.assertEqual(self.core.snapshot()['phase'], 'READY')
        current = self.core.activate()
        self.assertEqual(current['point'], point(30.1))
        self.assertNotEqual(current['token'], old['token'])

    def test_new_task_while_paused_stays_paused_until_explicit_resume(self):
        self.accept()
        self.accept('PAUSE')
        self.core.mark_stopped()
        self.accept(task_id='task-2', final_goal=point(32.), via_points=[point(31.)])
        self.assertEqual(self.core.snapshot()['phase'], 'PAUSED')
        self.assertTrue(self.core.snapshot()['paused'])
        self.assertIsNone(self.core.activate())
        self.accept('RESUME', task_id='task-2')
        self.assertEqual(self.core.activate()['point'], point(31.))

    def test_route_replacement_while_paused_never_releases_pause(self):
        self.accept(via_points=[point(30.1)])
        self.accept('PAUSE')
        self.core.mark_stopped()
        self.accept('REPLACE_VIA_POINTS', via_points=[point(30.2)])
        self.assertEqual((self.core.snapshot()['paused'], self.core.snapshot()['phase']),
                         (True, 'PAUSED'))
        self.assertIsNone(self.core.activate())
        self.accept('RESUME')
        self.assertEqual(self.core.activate()['point'], point(30.2))

    def test_new_task_during_pausing_waits_for_stop_before_claiming_paused(self):
        self.accept(via_points=[point(30.1)])
        old = self.core.activate()
        self.accept('PAUSE')
        self.accept(task_id='task-2', final_goal=point(32.), via_points=[point(31.)])
        state = self.core.snapshot()
        self.assertEqual(state['phase'], 'PAUSING',
                         'a task update cannot stand in for Action termination and quiet stop')
        self.assertTrue(state['paused'])
        self.assertEqual(state['task_id'], 'task-2')
        self.assertEqual(state['remaining_via_points'], [point(31.)])
        self.assertFalse(self.core.settle(old['token'], True))
        self.assertIsNone(self.core.activate())
        self.core.mark_stopped()
        self.assertEqual(self.core.snapshot()['phase'], 'PAUSED')
        self.accept('RESUME', task_id='task-2')
        self.assertEqual(self.core.activate()['point'], point(31.))

    def test_route_update_during_pausing_waits_for_stop_before_claiming_paused(self):
        self.accept(final_goal=point(32.), via_points=[point(30.1)])
        old = self.core.activate()
        self.accept('PAUSE')
        self.accept('REPLACE_VIA_POINTS', via_points=[point(31.)])
        state = self.core.snapshot()
        self.assertEqual(state['phase'], 'PAUSING',
                         'route replacement must preserve the outstanding stop confirmation')
        self.assertTrue(state['paused'])
        self.assertEqual(state['final_goal'], point(32.))
        self.assertEqual(state['remaining_via_points'], [point(31.)])
        self.assertFalse(self.core.settle(old['token'], True))
        self.assertIsNone(self.core.activate())
        self.core.mark_stopped()
        self.assertEqual(self.core.snapshot()['phase'], 'PAUSED')
        self.accept('RESUME')
        self.assertEqual(self.core.activate()['point'], point(31.))

    def test_cancel_waits_for_stop_then_cannot_be_revived(self):
        self.accept(via_points=[point(30.1)])
        old = self.core.activate()
        self.accept('CANCEL')
        state = self.core.snapshot()
        self.assertEqual(state['phase'], 'CANCELING')
        self.assertEqual(state['remaining_via_points'], [point(30.1)])
        self.assertIsNone(self.core.activate())
        self.assertFalse(self.core.settle(old['token'], True))
        self.assertFalse(self.core.handle(self.command('RESUME'))['accepted'])
        self.core.mark_stopped()
        self.assertEqual(self.core.snapshot()['phase'], 'CANCELED')
        self.assertEqual(self.core.snapshot()['remaining_via_points'], [])
        self.assertEqual(self.core.snapshot()['last_task_result']['status'], 'CANCELED')
        for operation in ('RESUME', 'REPLACE_VIA_POINTS', 'PAUSE', 'CANCEL'):
            with self.subTest(operation=operation):
                result = self.core.handle(self.command(operation, via_points=[]))
                self.assertFalse(result['accepted'])
                self.assertIsNone(self.core.activate())

    def test_completed_task_rejects_late_updates_and_id_cannot_be_reused(self):
        self.accept()
        self.core.settle(self.core.activate()['token'], True)
        for operation in ('RESUME', 'REPLACE_VIA_POINTS', 'PAUSE', 'CANCEL', 'SET_TASK'):
            with self.subTest(operation=operation):
                self.assertFalse(self.core.handle(self.command(operation, via_points=[]))['accepted'])
        self.assertEqual(self.core.snapshot()['phase'], 'COMPLETED')
        self.accept(task_id='task-2')
        self.assertFalse(self.core.handle(self.command(task_id='task-1'))['accepted'])

    def test_active_task_id_cannot_be_reused_for_set_task(self):
        self.accept(final_goal=point(31.))
        self.assertFalse(self.core.handle(self.command(final_goal=point(32.)))['accepted'])
        self.assertEqual(self.core.snapshot()['final_goal'], point(31.))

    def test_unreachable_via_blocks_without_skipping_and_resume_retries(self):
        self.accept(via_points=[point(30.1), point(30.2)])
        old = self.core.activate()
        self.assertTrue(self.core.settle(old['token'], False, 'no path'))
        state = self.core.snapshot()
        self.assertEqual((state['paused'], state['phase'], state['reason']), (True, 'BLOCKED', 'no path'))
        self.assertEqual(state['remaining_via_points'], [point(30.1), point(30.2)])
        self.assertIsNone(state['last_task_result'])
        self.assertIsNone(self.core.activate())
        self.accept('RESUME')
        self.assertEqual(self.core.activate()['point'], point(30.1))

    def test_final_failure_preserves_final_for_explicit_retry(self):
        self.accept()
        self.assertTrue(self.core.settle(self.core.activate()['token'], False, 'geographic check failed',
                                         result={'geographic': {'confirmed': False}}))
        self.assertEqual(self.core.snapshot()['phase'], 'BLOCKED')
        self.assertEqual(self.core.snapshot()['final_goal'], point())
        self.accept('RESUME')
        self.assertEqual(self.core.activate()['kind'], 'final')

    def test_external_block_invalidates_active_without_dropping_route(self):
        self.accept(via_points=[point(30.1)])
        old = self.core.activate()
        self.core.block('registration changed')
        self.assertFalse(self.core.settle(old['token'], True))
        self.assertEqual(self.core.snapshot()['remaining_via_points'], [point(30.1)])
        self.assertEqual(self.core.snapshot()['phase'], 'BLOCKED')
        self.core.mark_stopped()
        self.assertEqual(self.core.snapshot()['phase'], 'BLOCKED')
        self.assertIsNone(self.core.activate())

    def test_unknown_and_repeated_tokens_cannot_change_current_segment(self):
        self.accept(via_points=[point(30.1), point(30.2)])
        first = self.core.activate()
        self.assertFalse(self.core.settle('unknown', True))
        self.assertTrue(self.core.settle(first['token'], True))
        second = self.core.activate()
        self.assertFalse(self.core.settle(first['token'], True))
        self.assertEqual(self.core.snapshot()['active'], second)
        self.assertEqual(self.core.snapshot()['remaining_via_points'], [point(30.2)])

    def test_invalid_final_or_via_batch_is_atomic_and_consumes_its_sequence(self):
        self.accept(via_points=[point(30.1)])
        active = self.core.activate()
        before = self.core.snapshot()
        invalid = self.command('REPLACE_VIA_POINTS', via_points=[point(30.3), point(91.)])
        result = self.core.handle(invalid)
        self.assertFalse(result['accepted'])
        state = self.core.snapshot()
        for key in ('task_id', 'revision', 'phase', 'paused', 'final_goal', 'remaining_via_points', 'active'):
            self.assertEqual(state[key], before[key], key)
        self.assertEqual(state['last_sequence'], invalid['sequence'])
        self.assertEqual(state['active'], active)
        invalid_final = self.command(task_id='task-2', final_goal=point(longitude=181.))
        self.assertFalse(self.core.handle(invalid_final)['accepted'])
        self.assertEqual(self.core.snapshot()['task_id'], 'task-1')
        repaired = copy.deepcopy(invalid)
        repaired.update(request_id='different-request', via_points=[point(30.3)])
        self.assertFalse(self.core.handle(repaired)['accepted'], 'invalid batch sequence remains consumed')

    def test_duplicate_accepted_command_returns_original_result_without_reexecution(self):
        command = self.command(via_points=[point(30.1)])
        original = self.core.handle(command)
        self.assertTrue(original['accepted'])
        first = self.core.activate()
        self.core.settle(first['token'], True)
        before = self.core.snapshot()
        duplicate = self.core.handle(copy.deepcopy(command))
        expected = dict(original, duplicate=True)
        self.assertEqual(duplicate, expected)
        self.assertEqual(self.core.snapshot(), before)
        self.assertEqual(self.core.activate()['kind'], 'final')

    def test_duplicate_rejection_stays_rejected_after_other_commands(self):
        invalid = self.command(final_goal=point(91.))
        original = self.core.handle(invalid)
        self.assertFalse(original['accepted'])
        self.accept(task_id='task-2')
        before = self.core.snapshot()
        self.assertEqual(self.core.handle(invalid), dict(original, duplicate=True))
        self.assertEqual(self.core.snapshot(), before)

    def test_same_request_id_with_different_content_rejects_without_consuming_sequence(self):
        command = self.command()
        self.assertTrue(self.core.handle(command)['accepted'])
        conflicting = copy.deepcopy(command)
        conflicting.update(sequence=100, final_goal=point(31.))
        result = self.core.handle(conflicting)
        self.assertFalse(result['accepted'])
        self.assertFalse(result['duplicate'])
        self.assertEqual(self.core.snapshot()['last_sequence'], 1)
        self.assertEqual(self.core.snapshot()['final_goal'], point())
        self.accept('PAUSE')

    def test_sequence_is_global_allows_skips_and_rejects_old_after_task_end(self):
        self.accept(sequence=20)
        self.assertFalse(self.core.handle(self.command('PAUSE', sequence=19))['accepted'])
        self.accept('CANCEL', sequence=80)
        self.core.mark_stopped()
        self.assertFalse(self.core.handle(self.command(task_id='task-2', sequence=40))['accepted'])
        self.assertEqual(self.core.snapshot()['phase'], 'CANCELED')
        self.assertEqual(self.core.snapshot()['last_sequence'], 80)
        self.accept(task_id='task-2', sequence=81)

    def test_wrong_boot_is_rejected_without_poisoning_sequence(self):
        self.assertFalse(self.core.handle(self.command(server_id='old-boot', sequence=500))['accepted'])
        self.assertIsNone(self.core.snapshot()['last_sequence'])
        self.accept(sequence=1)
        self.assertEqual(self.core.snapshot()['last_sequence'], 1)

    def test_zero_sequence_is_rejected_without_consuming_boot_watermark(self):
        self.assertFalse(self.core.handle(self.command(sequence=0))['accepted'])
        self.assertIsNone(self.core.snapshot()['last_sequence'])
        self.accept(sequence=1)

    def test_rejected_wrong_task_consumes_sequence_but_does_not_change_route(self):
        self.accept()
        result = self.core.handle(self.command('CANCEL', task_id='other', sequence=50))
        self.assertFalse(result['accepted'])
        self.assertEqual(self.core.snapshot()['last_sequence'], 50)
        self.assertEqual(self.core.snapshot()['phase'], 'READY')
        self.assertFalse(self.core.handle(self.command('PAUSE', sequence=49))['accepted'])

    def test_receipt_replays_original_result_after_many_later_commands(self):
        first = self.command()
        original = self.core.handle(first)
        for number in range(301):
            command = self.command('PAUSE' if number % 2 == 0 else 'RESUME')
            self.assertTrue(self.core.handle(command)['accepted'])
        before = self.core.snapshot()
        result = self.core.handle(first)
        self.assertEqual(result, dict(original, duplicate=True))
        self.assertEqual(self.core.snapshot(), before)

    def test_old_request_id_cannot_bind_different_content_after_many_later_commands(self):
        first = self.command()
        self.assertTrue(self.core.handle(first)['accepted'])
        for number in range(301):
            command = self.command('PAUSE' if number % 2 == 0 else 'RESUME')
            self.assertTrue(self.core.handle(command)['accepted'])
        before = self.core.snapshot()
        different = dict(first, sequence=1000, task_id='task-2', final_goal=point(32.))
        result = self.core.handle(different)
        self.assertFalse(result['accepted'], 'later sequence cannot reuse a previously bound request_id')
        self.assertFalse(result['duplicate'])
        self.assertEqual(self.core.snapshot(), before)

    def test_point_validation_rejects_nonfinite_boolean_ranges_and_via_yaw(self):
        invalid_points = [point(math.nan), point(math.inf), point(True), point(-91.),
                          point(longitude=-181.), point(altitude=math.nan), point(altitude=False),
                          point(yaw=math.inf), point(yaw=True), {'latitude': 30.},
                          dict(point(), extra=2), [], None]
        for index, bad in enumerate(invalid_points):
            with self.subTest(point=bad):
                self.assertFalse(self.core.handle(self.command(task_id='bad-{}'.format(index),
                                                               final_goal=bad))['accepted'])
                self.assertIsNone(self.core.snapshot()['task_id'])
        self.assertFalse(self.core.handle(self.command(via_points=[point(yaw=0.)]))['accepted'])
        self.accept(final_goal=point(-90., -180., -30., -math.pi),
                    via_points=[point(90., 180.)])

    def test_missing_optional_point_values_preserve_unspecified_height_and_yaw(self):
        self.accept(final_goal={'latitude': 30., 'longitude': 120.},
                    via_points=[{'latitude': 31., 'longitude': 121.}])
        self.assertEqual(self.core.snapshot()['final_goal'], point())
        self.assertEqual(self.core.activate()['point'], point(31., 121.))

    def test_invalid_envelopes_do_not_consume_sequence_or_mutate_task(self):
        valid = self.command()
        mutations = [dict(protocol_version=True), dict(protocol_version=2), dict(server_id=''),
                     dict(sequence=True), dict(sequence=-1), dict(sequence=2**64),
                     dict(sequence=1.), dict(request_id=''), dict(task_id=''),
                     dict(operation='set_task'), dict(operation='UNKNOWN'), dict(extra='unknown')]
        for index, updates in enumerate(mutations):
            with self.subTest(updates=updates):
                candidate = dict(valid, request_id='invalid-envelope-{}'.format(index))
                candidate.update(updates)
                self.assertFalse(self.core.handle(candidate)['accepted'])
                self.assertIsNone(self.core.snapshot()['last_sequence'])
        for candidate in (None, [], 'invalid'):
            self.assertFalse(self.core.handle(candidate)['accepted'])
        self.assertTrue(self.core.handle(valid)['accepted'])

    def test_invalid_envelope_receipt_binds_request_id_without_consuming_sequence(self):
        for updates in (dict(protocol_version=2), dict(operation='UNKNOWN'), dict(sequence=0)):
            with self.subTest(updates=updates):
                self.core = TaskCore('boot-1')
                invalid = self.command()
                invalid.update(updates)
                rejected = self.core.handle(invalid)
                self.assertFalse(rejected['accepted'])
                self.assertIsNone(self.core.snapshot()['last_sequence'])
                self.assertEqual(self.core.handle(copy.deepcopy(invalid)), dict(rejected, duplicate=True))
                corrected = dict(invalid, protocol_version=1, operation='SET_TASK',
                                 sequence=max(invalid['sequence'], 1))
                conflict = self.core.handle(corrected)
                self.assertFalse(conflict['accepted'], 'repairing content requires a new request_id')
                self.assertFalse(conflict['duplicate'])
                self.assertIsNone(self.core.snapshot()['last_sequence'])
                self.assertIsNone(self.core.snapshot()['task_id'])
                corrected['request_id'] += '-new'
                self.assertTrue(self.core.handle(corrected)['accepted'])

    def test_identifiable_incomplete_envelope_rejection_is_replayable(self):
        invalid = self.command()
        del invalid['operation']
        del invalid['task_id']
        rejected = self.core.handle(invalid)
        self.assertFalse(rejected['accepted'])
        self.assertIsNone(self.core.snapshot()['last_sequence'])
        self.assertEqual(self.core.handle(invalid), dict(rejected, duplicate=True))
        corrected = dict(invalid, operation='SET_TASK', task_id='task-1')
        self.assertFalse(self.core.handle(corrected)['accepted'])
        self.assertIsNone(self.core.snapshot()['task_id'])

    def test_wrong_receiver_boot_does_not_bind_request_id_in_this_boot(self):
        wrong_boot = self.command(server_id='old-boot')
        self.assertFalse(self.core.handle(wrong_boot)['accepted'])
        corrected = dict(wrong_boot, server_id='boot-1')
        self.assertTrue(self.core.handle(corrected)['accepted'])

    def test_non_json_envelope_cannot_bind_request_id(self):
        valid = self.command()
        invalid = dict(valid, extra=object())
        self.assertFalse(self.core.handle(invalid)['accepted'])
        self.assertIsNone(self.core.snapshot()['last_sequence'])
        self.assertTrue(self.core.handle(valid)['accepted'])

    def test_route_payload_shape_and_unused_payload_are_validated(self):
        self.accept()
        before = self.core.snapshot()
        commands = [self.command('REPLACE_VIA_POINTS'),
                    self.command('REPLACE_VIA_POINTS', via_points=None),
                    self.command('REPLACE_VIA_POINTS', via_points='not a list'),
                    self.command('PAUSE', via_points=[point()]),
                    self.command('RESUME', final_goal=point()),
                    self.command(task_id='task-2', via_points=[point(), point(yaw=1.)])]
        for candidate in commands:
            self.assertFalse(self.core.handle(candidate)['accepted'])
        self.assertEqual(self.core.snapshot()['final_goal'], before['final_goal'])
        self.assertEqual(self.core.snapshot()['phase'], 'READY')

    def test_snapshots_inputs_outputs_and_result_receipts_are_independent_copies(self):
        command = self.command(via_points=[point(30.1)])
        self.assertTrue(self.core.handle(command)['accepted'])
        command['via_points'][0]['latitude'] = 80.
        state = self.core.snapshot()
        state['final_goal']['latitude'] = 81.
        state['remaining_via_points'].clear()
        segment = self.core.activate()
        self.assertEqual(segment['point'], point(30.1))
        segment['point']['latitude'] = 82.
        self.assertEqual(self.core.snapshot()['active']['point'], point(30.1))
        self.core.settle(segment['token'], True)
        final = self.core.activate()
        review = {'geographic': {'confirmed': True}}
        self.core.settle(final['token'], True, result=review)
        review['geographic']['confirmed'] = False
        self.assertTrue(self.core.snapshot()['last_task_result']['result']['geographic']['confirmed'])

    def test_no_task_pause_resume_cancel_or_route_update_can_create_task(self):
        for operation in ('PAUSE', 'RESUME', 'CANCEL', 'REPLACE_VIA_POINTS'):
            with self.subTest(operation=operation):
                self.assertFalse(self.core.handle(self.command(operation, via_points=[]))['accepted'])
                self.assertIsNone(self.core.activate())
        self.core.block('no active task')
        self.core.mark_stopped()
        self.assertEqual(self.core.snapshot()['phase'], 'IDLE')

    def test_navigation_validator_rejects_set_or_replace_before_task_mutation(self):
        self.accept(via_points=[point(30.1)])
        self.core.activate()
        before = self.core.snapshot()

        def conversion_unavailable(final_goal, via_points):
            raise ValueError('target outside current map or registration unavailable')

        commands = [self.command(task_id='task-2', final_goal=point(31.)),
                    self.command('REPLACE_VIA_POINTS', via_points=[point(30.3)])]
        for command in commands:
            with self.subTest(operation=command['operation']):
                result = self.validate(command, conversion_unavailable)
                self.assertFalse(result['accepted'])
                self.assertIn('registration unavailable', result['reason'])
                state = self.core.snapshot()
                for key in ('task_id', 'revision', 'phase', 'paused', 'final_goal',
                            'remaining_via_points', 'active', 'last_task_result'):
                    self.assertEqual(state[key], before[key], key)
                self.assertEqual(state['last_sequence'], command['sequence'])

    def test_navigation_validator_receives_complete_candidate_and_cannot_mutate_it(self):
        def validate_new(final_goal, via_points):
            self.assertEqual(final_goal, point(31.))
            self.assertEqual(via_points, [point(30.1)])
            final_goal['latitude'] = 80.
            via_points.clear()

        command = self.command(final_goal=point(31.), via_points=[point(30.1)])
        self.assertTrue(self.validate(command, validate_new)['accepted'])
        self.assertEqual(self.core.snapshot()['final_goal'], point(31.))
        self.assertEqual(self.core.snapshot()['remaining_via_points'], [point(30.1)])

        def validate_replacement(final_goal, via_points):
            self.assertEqual(final_goal, point(31.))
            self.assertEqual(via_points, [point(30.2)])

        self.assertTrue(self.validate(self.command('REPLACE_VIA_POINTS', via_points=[point(30.2)]),
                                      validate_replacement)['accepted'])

    def test_duplicate_receipt_bypasses_current_navigation_validator(self):
        command = self.command()
        original = self.validate(command, lambda final, vias: None)
        self.assertTrue(original['accepted'])

        def changed_registration(final_goal, via_points):
            raise AssertionError('duplicate must replay receipt without validating or executing again')

        self.assertEqual(self.validate(command, changed_registration), dict(original, duplicate=True))
        invalid = self.command('REPLACE_VIA_POINTS', via_points=[])

        def reject(final_goal, via_points):
            raise ValueError('registration unavailable')

        original = self.validate(invalid, reject)
        self.assertFalse(original['accepted'])
        self.assertEqual(self.validate(invalid, changed_registration), dict(original, duplicate=True))

    def test_pause_and_cancel_do_not_require_fresh_navigation_validation(self):
        self.accept()

        def unavailable(final_goal, via_points):
            raise AssertionError('stop command must not require geographic localization')

        self.assertTrue(self.validate(self.command('PAUSE'), unavailable)['accepted'])
        self.core.mark_stopped()
        self.assertTrue(self.validate(self.command('CANCEL'), unavailable)['accepted'])
        self.core.mark_stopped()
        self.assertEqual(self.core.snapshot()['phase'], 'CANCELED')

    def test_resume_validates_retained_route_and_failure_keeps_task_paused(self):
        self.accept(final_goal=point(31.), via_points=[point(30.1), point(30.2)])
        self.core.settle(self.core.activate()['token'], True)
        self.accept('PAUSE')
        self.core.mark_stopped()
        before = self.core.snapshot()

        def reject(final_goal, via_points):
            self.assertEqual(final_goal, point(31.))
            self.assertEqual(via_points, [point(30.2)])
            raise ValueError('registration reset')

        command = self.command('RESUME')
        self.assertFalse(self.validate(command, reject)['accepted'])
        state = self.core.snapshot()
        self.assertEqual((state['phase'], state['paused'], state['revision']),
                         ('PAUSED', True, before['revision']))
        self.assertEqual(state['remaining_via_points'], [point(30.2)])
        self.assertEqual(state['last_sequence'], command['sequence'])
        self.assertIsNone(self.core.activate())
        self.assertTrue(self.validate(self.command('RESUME'), lambda final, vias: None)['accepted'])
        self.assertEqual(self.core.activate()['point'], point(30.2))


if __name__ == '__main__':
    unittest.main()
