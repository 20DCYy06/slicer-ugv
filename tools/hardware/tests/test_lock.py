import sys
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

TOOLS = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(TOOLS))
from hardware_path import hardware_root

HARDWARE = hardware_root(TOOLS.parent)
sys.path.insert(0, str(HARDWARE / 'LockDown'))
import lock


class LockTests(unittest.TestCase):
    def test_cli_uses_the_owner_without_opening_serial(self):
        with patch.object(lock, 'Chassis') as chassis, patch.object(lock, 'save_log'), \
             patch.object(lock, 'call_owner', return_value=(True, 'confirmed')) as owner, \
             patch.object(sys, 'argv', ['lock.py']):
            self.assertEqual(lock.main(), 0)
        chassis.assert_not_called()
        owner.assert_called_once_with('5CF0016377', 'lock')

    def test_idle_lock_waits_for_arming_and_confirms_firmware_latch(self):
        state = dict(motor_power_present=True, motor_enabled=True, tick=123, ack=0, fault_latched=False,
                     wheel_mm_s=[0]*4, pwm_ticks=[0]*4)
        car = Mock(nonce=123, state=state, body={'status': 0x47, 'tick': 123})
        car.read_state.return_value = state

        def send(kind, request):
            if kind == 0x10:
                state['ack'] = 10
                return 10
            if request[-1] == 2:
                car.body['status'] |= 0x10
            else:
                car.body['status'] |= 0x2000
                state['fault_latched'] = True

        car.send.side_effect = send
        car.wait.side_effect = lambda condition, _timeout: self.assertTrue(condition())
        lock.lock_standalone(car)
        self.assertEqual([(call.args[0], call.args[1][-1]) for call in car.send.call_args_list],
                         [(0x19,2),(0x10,0),(0x19,1)])

    def test_idle_lock_refuses_nonzero_pwm_before_handshake(self):
        car = Mock()
        car.read_state.return_value = dict(motor_power_present=True, motor_enabled=True,
                                           wheel_mm_s=[0]*4, pwm_ticks=[1,0,0,0])
        with self.assertRaises(RuntimeError):
            lock.lock_standalone(car)
        car.connect.assert_not_called()
        car.send.assert_not_called()
