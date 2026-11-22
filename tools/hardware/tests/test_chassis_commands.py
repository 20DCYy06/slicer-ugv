import os
import struct
import sys
import time
import unittest
from pathlib import Path
from unittest.mock import Mock

TOOLS = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(TOOLS))
from hardware_path import hardware_root

HARDWARE = hardware_root(TOOLS.parent)
sys.path.insert(0, str(HARDWARE / 'Host'))
from chassis_commands import CommandServer, call_owner
from chassis_recovery import LinkRecovery


class ChassisCommandTests(unittest.TestCase):
    def test_commands_reach_only_the_matching_local_owner(self):
        serial = 'test-' + str(os.getpid())
        self.assertIsNone(call_owner(serial, 'unlock'))
        recovery = Mock()
        recovery.queue_unlock.return_value.result.return_value = (True, 'cleared')
        recovery.queue_test.return_value.result.return_value = (True, 'locked')
        with CommandServer(serial, recovery):
            self.assertEqual(call_owner(serial, 'unlock'), (True, 'cleared'))
            self.assertEqual(call_owner(serial, 'lock'), (True, 'locked'))
            self.assertIsNone(call_owner(serial + '-different', 'lock'))

    def test_driver_waits_for_cli_and_fresh_clear_confirmation(self):
        start = Mock()
        recovery = LinkRecovery()
        recovery.start_unlock_command = start
        car = Mock(nonce=123)
        recovery.update(car, 1., 0x3e, 0x10c7, stationary=True)
        start.assert_called_once()
        car.send.assert_not_called()
        request = recovery.queue_unlock()
        recovery.update(car, 1.1, 0x3e, 0x10c7, stationary=True)
        car.send.assert_called_once_with(0x19, struct.pack('<IB', 123, 0))
        self.assertFalse(request.done())
        self.assertEqual(recovery.update(car, 1.2, 0x3a, 0x47, stationary=True), (True, True))
        self.assertTrue(request.result()[0])

    def test_driver_spawns_real_unlock_cli_against_simulated_mcu(self):
        serial = 'auto-test-' + str(os.getpid())
        script = HARDWARE / 'Unlock/unlock.py'
        recovery = LinkRecovery()
        car = Mock(nonce=123)
        server = CommandServer(serial, recovery).start(script)
        try:
            deadline = time.monotonic() + 3.
            while not car.send.called and time.monotonic() < deadline:
                recovery.update(car, time.monotonic(), 0x3e, 0x10c7, stationary=True)
                time.sleep(.01)
            car.send.assert_called_once_with(0x19, struct.pack('<IB', 123, 0))
            self.assertEqual(recovery.update(car, time.monotonic(), 0x3a, 0x47, stationary=True),
                             (True, True))
            self.assertEqual(server.child.wait(timeout=2), 0)
        finally:
            server.close()

    def test_lock_requires_zero_output_and_a_stop_ack(self):
        recovery = LinkRecovery()
        car = Mock(nonce=123)
        car.send.return_value = 100
        request = recovery.queue_test()
        recovery.update(car, 1., 0x3a, 0x57, stationary=False)
        self.assertFalse(request.result()[0])
        car.send.assert_not_called()
        request = recovery.queue_test()
        self.assertEqual(recovery.update(car, 1.1, 0x3a, 0x57, stationary=True, ack=99), (True, False))
        car.send.assert_called_once_with(0x10, bytes(8))
        recovery.update(car, 1.2, 0x3a, 0x57, stationary=True, ack=101)
        self.assertEqual(car.send.call_count, 1)
        recovery.update(car, 1.3, 0x3a, 0x57, stationary=True, ack=100)
        car.send.assert_called_with(0x19, struct.pack('<IB', 123, 1))
        self.assertFalse(request.done())
        recovery.update(car, 1.4, 0x3e, 0x2057, stationary=True)
        self.assertTrue(request.result()[0])

    def test_shutdown_rejects_pending_and_late_commands(self):
        recovery = LinkRecovery()
        car = Mock(nonce=123)
        test = recovery.queue_test()
        recovery.update(car, 1., 0x3a, 0x57, stationary=True)
        unlock = recovery.queue_unlock()
        recovery.stop_requests()
        self.assertFalse(test.result()[0])
        self.assertFalse(unlock.result()[0])
        self.assertFalse(recovery.queue_test().result()[0])
