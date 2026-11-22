#!/usr/bin/env python3
"""The real task core and executor, with finite deterministic Action/Mux I/O."""
import sys
import unittest
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from air_ground_core import TaskCore
try:
    from air_ground_executor import TaskExecutor
except ImportError:
    TaskExecutor = None


def point(x=1., y=2.):
    return dict(latitude=x, longitude=y, altitude_ellipsoid_m=None, yaw=None)


class Backend:
    def __init__(self):
        self.goals, self.cancels, self.holds, self.releases = [], 0, 0, 0
        self.state, self.xy, self.quiet, self.bad_point = 1, (0., 0.), True, None
        self.review_result = None
        self.prep_required = False
        self.prep_phase, self.prep_reason = 'WAITING_RTK', ''
        self.prep_starts, self.prep_cancels = [], 0
        self.mux = dict(manual_hold=False, emergency_stop_latched=False, teleop_fresh=False,
                        mux_instance_id='mux', takeover_generation=0)

    def validate_route(self, final, vias):
        if self.bad_point in [p['latitude'] for p in [final]+vias]:
            raise ValueError('conversion unavailable')

    def needs_preparation(self):
        return self.prep_required

    def prepare_start(self, task_id, target):
        self.prep_starts.append((task_id, target))

    def prepare_poll(self):
        return self.prep_phase, self.prep_reason

    def prepare_cancel(self):
        self.prep_cancels += 1

    def active_server_goal(self):
        return bool(self.goals) and self.state not in (2, 3, 4, 5, 8)

    def target(self, p):
        return dict(map_xyz=[p['latitude'], p['longitude'], 0.], yaw=1., frame='map',
                    generation=1, original=p)

    def health(self, target=None):
        return None

    def mux_state(self):
        return dict(self.mux)

    def start(self, target):
        self.goals.append(target)
        self.state = 1

    def poll(self):
        return dict(state=self.state, text='fake Action')

    def cancel(self):
        self.cancels += 1  # Deliberately not an immediate Action acknowledgment.

    def position(self):
        return self.xy

    def hold(self):
        self.holds += 1
        self.mux.update(manual_hold=True, takeover_generation=self.mux['takeover_generation']+1)
        return dict(self.mux)

    def release(self, expected_hold=None):
        if expected_hold is not None and expected_hold != (self.mux['mux_instance_id'], self.mux['takeover_generation']):
            raise ValueError('external hold changed before release')
        self.releases += 1
        self.mux['manual_hold'] = False
        return dict(self.mux)

    def stopped(self, since):
        return dict(confirmed=self.quiet, reason='' if self.quiet else 'still moving')

    def review(self, target, action, since):
        return self.review_result or dict(move_base=action, stop={'confirmed': False},
                                         geographic={'confirmed': False, 'qualified': False})


class Availability(unittest.TestCase):
    def test_executor_exists(self):
        self.assertIsNotNone(TaskExecutor, 'TaskExecutor is not implemented')


@unittest.skipIf(TaskExecutor is None, 'implementation not present yet')
class ExecutorTests(unittest.TestCase):
    def setUp(self):
        self.now = 0.
        self.backend = Backend()
        self.executor = TaskExecutor(TaskCore('boot'), self.backend, clock=lambda: self.now)
        self.seq = 0

    def command(self, operation='SET_TASK', task='task', **payload):
        self.seq += 1
        cmd = dict(protocol_version=1, server_id='boot', request_id=str(self.seq),
                   sequence=self.seq, task_id=task, operation=operation, **payload)
        if operation == 'SET_TASK':
            cmd.setdefault('final_goal', point())
            cmd.setdefault('via_points', [])
        result = self.executor.command(cmd)
        self.assertTrue(result['accepted'], result)
        self.executor.tick()
        return cmd

    def test_first_task_uses_latest_intent_only_after_preparation(self):
        self.backend.prep_required = True
        self.command(via_points=[point(3., 4.)])
        self.assertEqual(self.backend.prep_starts, [('task', point())])
        self.backend.prep_phase = 'PREPARING'
        self.executor.tick()
        self.command('SET_TASK', task='replacement', final_goal=point(7., 8.),
                     via_points=[point(5., 6.)])
        state = self.executor.snapshot()
        self.assertEqual(state['runtime_phase'], 'PREPARING')
        self.assertIsNone(state['active'])
        self.assertEqual(state['remaining_via_points'], [point(5., 6.)])
        self.assertEqual(self.backend.goals, [])
        self.backend.prep_phase = 'SWITCHING_NAVIGATION'
        self.command('REPLACE_VIA_POINTS', task='replacement', via_points=[point(8., 9.)])
        self.assertEqual(self.backend.goals, [])
        self.backend.prep_phase = 'READY'
        self.backend.prep_required = False
        self.executor.tick()
        self.executor.tick()
        self.assertEqual(len(self.backend.prep_starts), 1)
        self.assertEqual(self.backend.prep_cancels, 0)
        self.assertEqual([goal['original'] for goal in self.backend.goals], [point(8., 9.)])

    def test_pause_during_preparation_waits_for_stop_and_resumes_with_owned_hold(self):
        self.backend.prep_required = True
        self.command()
        self.backend.prep_phase = 'PREPARING'
        self.command('PAUSE')
        self.assertEqual(self.backend.prep_cancels, 1)
        self.assertEqual(self.executor.snapshot()['phase'], 'PAUSING')
        self.assertEqual(self.backend.goals, [])
        self.backend.prep_phase = 'STOPPED'
        self.executor.tick()
        self.executor.tick()
        self.assertEqual(self.executor.snapshot()['phase'], 'PAUSED')
        self.assertTrue(self.backend.mux['manual_hold'])
        self.backend.prep_phase = 'WAITING_RTK'
        self.command('RESUME')
        self.assertEqual(self.backend.releases, 1)
        self.assertEqual(len(self.backend.prep_starts), 2)

    def test_cancel_during_switch_does_not_dispatch_even_if_ready_arrives(self):
        self.backend.prep_required = True
        self.command()
        self.backend.prep_phase = 'SWITCHING_NAVIGATION'
        self.command('CANCEL')
        self.assertEqual(self.backend.prep_cancels, 1)
        self.backend.prep_phase = 'READY'
        self.backend.prep_required = False
        self.executor.tick()
        self.executor.tick()
        self.assertEqual(self.executor.snapshot()['phase'], 'CANCELED')
        self.assertEqual(self.backend.goals, [])

    def test_preparation_failure_preserves_route_without_canceling_unowned_action(self):
        self.backend.prep_required = True
        self.command()
        self.backend.prep_phase, self.backend.prep_reason = 'FAILED', 'external goal in progress'
        self.executor.tick()
        self.executor.tick()
        state = self.executor.snapshot()
        self.assertEqual(state['phase'], 'BLOCKED')
        self.assertEqual(state['last_error'], 'external goal in progress')
        self.assertEqual(state['final_goal'], point())
        self.assertEqual(self.backend.holds, 0)
        self.assertEqual(self.backend.cancels, 0)

    def test_duplicate_does_not_dispatch_twice(self):
        cmd = self.command()
        self.executor.command(cmd)
        self.executor.tick()
        self.assertEqual(len(self.backend.goals), 1)

    def test_transport_exception_rejects_batch_without_destroying_active_task(self):
        self.command()
        old = self.executor.core.snapshot()
        def missing_tf(final, vias):
            raise RuntimeError('TF temporarily unavailable')
        self.backend.validate_route = missing_tf
        cmd = dict(protocol_version=1, server_id='boot', request_id='tf', sequence=2,
                   task_id='replacement', operation='SET_TASK', final_goal=point(), via_points=[])
        receipt = self.executor.command(cmd)
        self.assertFalse(receipt['accepted'])
        self.assertEqual(self.executor.core.snapshot()['task_id'], old['task_id'])
        self.assertEqual(self.backend.cancels, 0)

    def test_owned_hold_release_binds_generation_even_if_external_hold_arrives_during_rpc(self):
        self.command()
        self.command('PAUSE')
        self.backend.state = 2
        self.executor.tick()
        original_release = self.backend.release
        def raced_release(expected_hold=None):
            self.backend.mux['takeover_generation'] += 1
            return original_release(expected_hold)
        self.backend.release = raced_release
        self.command('RESUME')
        self.assertEqual(self.backend.releases, 0)
        self.assertTrue(self.backend.mux['manual_hold'])
        self.assertEqual(len(self.backend.goals), 1)

    def test_via_position_pass_waits_for_cancel_ack_before_next_goal(self):
        self.command(via_points=[point(3., 4.)])
        self.backend.xy = (3.1, 4.)
        self.executor.tick()
        self.assertEqual(self.backend.cancels, 1)
        self.assertEqual(len(self.backend.goals), 1)
        self.executor.tick()
        self.assertEqual(len(self.backend.goals), 1)
        self.backend.state = 2
        self.executor.tick()
        self.executor.tick()
        self.assertEqual(len(self.backend.goals), 2)
        self.assertEqual(self.backend.goals[-1]['original'], point())
        self.assertEqual(self.backend.holds, 0, 'via is not a parking/geographic review')

    def test_replacement_late_old_success_does_not_skip_new_via(self):
        self.command(via_points=[point(3., 4.)])
        self.command('REPLACE_VIA_POINTS', via_points=[point(5., 6.)])
        self.assertEqual(len(self.backend.goals), 1)
        self.backend.state = 3  # An old SUCCEEDED arriving after replacement.
        self.executor.tick()
        self.executor.tick()
        self.assertEqual(self.backend.goals[-1]['original'], point(5., 6.))
        self.assertEqual(len(self.executor.core.snapshot()['remaining_via_points']), 1)

    def test_cancel_timeout_blocks_and_never_starts_replacement(self):
        self.command()
        self.command('SET_TASK', task='new')
        self.now = 4.
        self.executor.tick()
        self.assertEqual(len(self.backend.goals), 1)
        self.assertEqual(self.executor.core.snapshot()['phase'], 'BLOCKED')

    def test_pause_requires_ack_and_quiet_and_update_stays_paused(self):
        self.command()
        self.backend.quiet = False
        self.command('PAUSE')
        self.assertEqual(self.executor.core.snapshot()['phase'], 'PAUSING')
        self.backend.state = 2
        self.executor.tick()
        self.assertEqual(self.executor.core.snapshot()['phase'], 'PAUSING')
        self.backend.quiet = True
        self.executor.tick()
        self.assertEqual(self.executor.core.snapshot()['phase'], 'PAUSED')
        self.command('SET_TASK', task='new', final_goal=point(8., 9.))
        self.assertEqual(self.executor.core.snapshot()['phase'], 'PAUSED')
        self.assertEqual(len(self.backend.goals), 1)
        self.command('RESUME', task='new')
        self.assertEqual(len(self.backend.goals), 2)
        self.assertEqual(self.backend.releases, 1)

    def test_external_hold_is_never_released_by_task_update(self):
        self.backend.mux['manual_hold'] = True
        self.command()
        self.assertEqual(len(self.backend.goals), 0)
        self.assertEqual(self.backend.releases, 0)
        self.assertEqual(self.executor.core.snapshot()['phase'], 'BLOCKED')

    def test_external_generation_after_pause_cannot_be_released(self):
        self.command()
        self.command('PAUSE')
        self.backend.state = 2
        self.executor.tick()
        self.backend.mux['takeover_generation'] += 1
        self.command('RESUME')
        self.assertEqual(self.backend.releases, 0)
        self.assertEqual(self.executor.core.snapshot()['phase'], 'BLOCKED')

    def test_final_requires_separate_action_stop_geographic_facts(self):
        self.command()
        self.backend.state = 3
        self.executor.tick()
        self.assertNotEqual(self.executor.core.snapshot()['phase'], 'COMPLETED')
        self.backend.review_result = dict(move_base={'state': 3}, stop={'confirmed': True},
                                         geographic={'qualified': True, 'confirmed': True})
        self.executor.tick()
        self.assertEqual(self.executor.core.snapshot()['phase'], 'COMPLETED')
        self.assertEqual(self.backend.holds, 1)

    def test_invalid_batch_conversion_does_not_cancel_existing_goal(self):
        self.command(via_points=[point(3., 4.)])
        old = self.executor.core.snapshot()
        self.backend.bad_point = 8.
        cmd = dict(protocol_version=1, server_id='boot', request_id='bad', sequence=2,
                   task_id='task', operation='REPLACE_VIA_POINTS', via_points=[point(8., 9.)])
        self.assertFalse(self.executor.command(cmd)['accepted'])
        self.executor.tick()
        self.assertEqual(self.executor.core.snapshot()['remaining_via_points'], old['remaining_via_points'])
        self.assertEqual(self.backend.cancels, 0)

    def test_cancel_clears_only_after_confirmed_stop_and_cannot_resume(self):
        self.command(via_points=[point(3., 4.)])
        self.command('CANCEL')
        self.assertEqual(self.executor.core.snapshot()['phase'], 'CANCELING')
        self.backend.state = 2
        self.executor.tick()
        self.assertEqual(self.executor.core.snapshot()['phase'], 'CANCELED')
        self.assertEqual(self.executor.core.snapshot()['remaining_via_points'], [])
        cmd = dict(protocol_version=1, server_id='boot', request_id='late', sequence=3,
                   task_id='task', operation='RESUME')
        self.assertFalse(self.executor.command(cmd)['accepted'])


if __name__ == '__main__':
    unittest.main()
