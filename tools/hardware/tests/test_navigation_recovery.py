import struct
import sys
import unittest
from pathlib import Path
from unittest.mock import Mock

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / 'Hardware/Host'))
from chassis_recovery import LinkRecovery


class NavigationRecoveryTests(unittest.TestCase):
    def setUp(self):
        self.car = Mock(nonce=123)
        self.recovery = LinkRecovery()

    def test_waits_for_safe_firmware_status_and_discards_recovery_cycle(self):
        self.assertEqual(self.recovery.update(self.car, 0, 0x3e, 0x1047), (True, False))
        self.car.send.assert_not_called()
        self.assertEqual(self.recovery.update(self.car, .3, 0x3e, 0x10c7), (True, False))
        self.car.send.assert_called_once_with(0x19, struct.pack('<IB', 123, 0))
        self.assertEqual(self.recovery.update(self.car, .35, 0x3a, None), (True, False))
        self.assertEqual(self.recovery.update(self.car, .4, 0x3a, 0x47), (True, True))
        self.assertEqual(self.recovery.update(self.car, .45, 0x3a, 0x47), (False, False))

    def test_mixed_fault_is_never_cleared(self):
        self.recovery.update(self.car, 0, 0x3e, 0x12c7)
        self.car.send.assert_not_called()

    def test_test_injection_is_confirmed_by_fresh_fault_telemetry(self):
        request = self.recovery.queue_test()
        self.assertEqual(self.recovery.update(self.car, 0, 0x3a, 0x47), (True, False))
        self.car.send.assert_called_once_with(0x19, struct.pack('<IB', 123, 1))
        self.assertFalse(request.done())
        self.assertEqual(self.recovery.update(self.car, .1, 0x3e, 0x2047), (True, False))
        self.assertTrue(request.result()[0])

    def test_missing_test_confirmation_keeps_the_driver_stopped(self):
        request = self.recovery.queue_test()
        self.recovery.update(self.car, 0, 0x3a, 0x47)
        self.assertEqual(self.recovery.update(self.car, 1.1, 0x3a, None), (True, False))
        self.assertFalse(request.result()[0])
        self.assertEqual(self.recovery.update(self.car, 1.15, 0x3a, 0x47), (True, True))

    def test_watchdog_arming_is_retried_until_confirmed(self):
        recovery = LinkRecovery(arm_watchdog=True)
        for now, status in ((0,0x47),(.2,0x47),(.3,0x47),(.6,0x57)):
            recovery.update(self.car, now, 0x3a, status)
        self.assertEqual(self.car.send.call_count, 2)
