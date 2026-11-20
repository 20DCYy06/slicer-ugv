#!/usr/bin/env python3
"""Stop and Action-ownership regressions, with no ROS or device I/O."""
from pathlib import Path
import sys
import unittest

HERE = Path(__file__).resolve().parent
sys.path[:0] = [str(HERE.parent / 'scripts'), str(HERE)]

from air_ground_core import TaskCore
from air_ground_executor import TaskExecutor
from test_air_ground_executor import Backend, point


class CancelFailsBackend(Backend):
    def cancel(self):
        self.cancels += 1
        raise ValueError('Action cancellation transport unavailable')


class PartialDispatchBackend(Backend):
    def __init__(self):
        super().__init__()
        self.fail_after_start = True

    def start(self, target):
        super().start(target)
        if self.fail_after_start:
            raise ValueError('transport failed after dispatch')


class ConversionFailsBackend(Backend):
    def target(self, raw_point):
        raise ValueError('registration changed before dispatch')


class StopBoundaryTests(unittest.TestCase):
    def setUp(self):
        self.now = 0.
        self.sequence = 0
        self.backend = Backend()
        self.executor = self.make_executor(self.backend)

    def make_executor(self, backend):
        return TaskExecutor(TaskCore('boot'), backend, clock=lambda: self.now)

    def command(self, operation='SET_TASK', **payload):
        self.sequence += 1
        command = dict(protocol_version=1, server_id='boot', sequence=self.sequence,
                       request_id=str(self.sequence), task_id='task', operation=operation)
        if operation == 'SET_TASK':
            command.update(final_goal=point(), via_points=[])
        command.update(payload)
        receipt = self.executor.command(command)
        self.assertTrue(receipt['accepted'], receipt)
        self.executor.tick()
        return receipt

    def test_pause_holds_immediately_while_action_ack_is_pending(self):
        self.command()
        self.backend.quiet = False
        self.command('PAUSE')
        self.assertEqual(self.executor.core.snapshot()['phase'], 'PAUSING')
        self.assertEqual(self.backend.cancels, 1)
        self.assertTrue(self.backend.mux['manual_hold'],
                        'pause must hold before the Action acknowledges cancellation')
        self.assertEqual(len(self.backend.goals), 1)
        self.now = 2.
        self.executor.tick()
        self.assertEqual(self.executor.core.snapshot()['phase'], 'PAUSING')
        self.assertTrue(self.backend.mux['manual_hold'])
        self.assertEqual(self.backend.releases, 0)

    def test_cancel_holds_immediately_without_claiming_ack_or_quiet_stop(self):
        self.command(via_points=[point(3., 4.)])
        self.backend.quiet = False
        self.command('CANCEL')
        state = self.executor.core.snapshot()
        self.assertEqual(state['phase'], 'CANCELING')
        self.assertEqual(state['remaining_via_points'], [point(3., 4.)])
        self.assertTrue(self.backend.mux['manual_hold'],
                        'cancel must hold before waiting for terminal Action state')
        self.backend.state = 2
        self.executor.tick()
        self.assertEqual(self.executor.core.snapshot()['phase'], 'CANCELING')
        self.backend.quiet = True
        self.executor.tick()
        self.assertEqual(self.executor.core.snapshot()['phase'], 'CANCELED')

    def test_cancel_transport_failure_cannot_skip_independent_mux_hold(self):
        self.backend = CancelFailsBackend()
        self.executor = self.make_executor(self.backend)
        self.command()
        self.command('PAUSE')
        self.assertTrue(self.backend.mux['manual_hold'],
                        'Mux hold remains available when Action cancel transport fails')
        self.assertEqual(self.executor.core.snapshot()['phase'], 'BLOCKED')
        self.assertIsNotNone(self.executor.active, 'unconfirmed Action must remain owned')
        self.assertEqual(len(self.backend.goals), 1)

    def test_shutdown_cancel_transport_failure_still_attempts_mux_hold(self):
        self.backend = CancelFailsBackend()
        self.executor = self.make_executor(self.backend)
        self.command()
        try:
            self.executor.shutdown()
        except ValueError:
            # Propagating the cancel failure is allowed, skipping hold is not.
            pass
        self.assertTrue(self.backend.mux['manual_hold'],
                        'shutdown must independently hold despite cancellation failure')
        self.assertGreaterEqual(self.backend.holds, 1)

    def test_partial_dispatch_exception_retains_cancel_barrier_before_retry(self):
        self.backend = PartialDispatchBackend()
        self.executor = self.make_executor(self.backend)
        self.command()
        self.assertEqual(self.executor.core.snapshot()['phase'], 'BLOCKED')
        self.assertEqual(len(self.backend.goals), 1)
        self.assertGreaterEqual(self.backend.cancels, 1,
                                'a start exception after dispatch still needs cancellation')
        self.assertIsNotNone(self.executor.active)
        self.assertTrue(self.backend.mux['manual_hold'])
        self.backend.fail_after_start = False
        self.command('RESUME')
        self.assertEqual(len(self.backend.goals), 1,
                         'retry must wait for the possibly dispatched Action terminal ACK')
        self.executor.tick()
        self.assertEqual(len(self.backend.goals), 1)
        self.backend.state = 2
        self.executor.tick()
        self.executor.tick()
        self.assertEqual(len(self.backend.goals), 2)

    def test_conversion_failure_before_dispatch_does_not_cancel_nonexistent_action(self):
        self.backend = ConversionFailsBackend()
        self.executor = self.make_executor(self.backend)
        self.command()
        self.assertEqual(self.executor.core.snapshot()['phase'], 'BLOCKED')
        self.assertEqual(self.backend.goals, [])
        self.assertEqual(self.backend.cancels, 0,
                         'ownership registration must occur after conversion and before dispatch')
        self.assertIsNone(self.executor.active)
        self.assertTrue(self.backend.mux['manual_hold'])

    def test_parking_geographic_review_timeout_holds_in_the_same_tick(self):
        self.command()
        self.backend.state = 3
        self.executor.tick()
        self.assertEqual(self.executor.runtime_phase, 'VERIFYING')
        self.now = 9.
        self.executor.tick()
        self.assertEqual(self.executor.core.snapshot()['phase'], 'BLOCKED')
        self.assertTrue(self.backend.mux['manual_hold'],
                        'an unconfirmed parking review must hold before tick returns')
        self.assertEqual(self.executor.core.snapshot()['final_goal'], point())
        self.assertEqual(len(self.backend.goals), 1)

    def test_terminal_action_failure_holds_immediately_and_preserves_route(self):
        for vias in ([], [point(3., 4.)]):
            with self.subTest(via=bool(vias)):
                self.backend = Backend()
                self.executor = self.make_executor(self.backend)
                self.command(via_points=vias)
                self.backend.state = 4
                self.executor.tick()
                state = self.executor.core.snapshot()
                self.assertEqual(state['phase'], 'BLOCKED')
                self.assertEqual(state['remaining_via_points'], vias)
                self.assertEqual(state['final_goal'], point())
                self.assertTrue(self.backend.mux['manual_hold'],
                                'terminal navigation failure must hold before tick returns')


if __name__ == '__main__':
    unittest.main()
