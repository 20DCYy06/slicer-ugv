import errno, shlex, stat, sys, unittest
from contextlib import ExitStack
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
TOOLS = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(TOOLS))
from hardware_path import hardware_root

HARDWARE = hardware_root(TOOLS.parent)
sys.path.insert(0, str(HARDWARE / 'Unlock'))
import unlock
class Fake:
    instances=[]
    initial={'motor_enabled':False,'pwm_ticks':[0]*4,'fault_latched':True}
    def __init__(self,cfg):self.connected=False;self.state=dict(self.initial);self.instances.append(self)
    def read_state(self):
        if self.connected:self.state={'motor_enabled':False,'pwm_ticks':[0]*4,'fault_latched':False}
        return self.state
    def connect(self):self.connected=True
    def close(self):pass
class UnlockTests(unittest.TestCase):
    def run_action(self,arg):
        with patch.object(unlock,'Chassis',Fake),patch.object(unlock,'save_log'),patch.object(unlock,'call_owner',return_value=None),patch.object(sys,'argv',['unlock.py',arg]):return unlock.main()
    def test_passive_check_never_handshakes(self):
        Fake.initial={'motor_enabled':False,'pwm_ticks':[0]*4,'fault_latched':True}
        self.assertEqual(self.run_action('--check'),2);self.assertFalse(Fake.instances[-1].connected)
    def test_motor_on_refuses_unlock(self):
        Fake.initial={'motor_enabled':True,'pwm_ticks':[0]*4,'fault_latched':True}
        self.assertEqual(self.run_action('--unlock'),3);self.assertFalse(Fake.instances[-1].connected)
    def test_motor_off_unlock(self):
        Fake.initial={'motor_enabled':False,'pwm_ticks':[0]*4,'fault_latched':True}
        self.assertEqual(self.run_action('--unlock'),0);self.assertTrue(Fake.instances[-1].connected)


class UnlockPermissionTests(unittest.TestCase):
    def run_denied_action(self, action, *, device_group=20, active_group=1000, account_groups=(1000, 20)):
        args = ['unlock.py', action, '--config', "现场配置 $(test) 'v1.6'.yaml"]
        denied = PermissionError(errno.EACCES, 'Permission denied', '/dev/ttyACM1')
        user = SimpleNamespace(pw_name='ugv', pw_gid=1000)
        device = SimpleNamespace(st_gid=device_group, st_mode=stat.S_IFCHR | 0o660)
        patches = (
            patch.object(sys, 'argv', args),
            patch.object(unlock, 'load_config', return_value={'serial': {'serial_number': 'TEST'}}),
            patch.object(unlock, 'call_owner', return_value=None),
            patch.object(unlock, 'Chassis', side_effect=denied),
            patch.object(unlock, 'save_log'),
            patch('grp.getgrnam', return_value=SimpleNamespace(gr_gid=20)),
            patch('pwd.getpwuid', return_value=user),
            patch('os.getegid', return_value=active_group),
            patch('os.getgroups', return_value=[1000]),
            patch('os.getgrouplist', return_value=account_groups),
            patch('os.stat', return_value=device),
        )
        with ExitStack() as stack:
            for mock in patches:
                stack.enter_context(mock)
            execute = stack.enter_context(patch('os.execlp', side_effect=SystemExit(0)))
            try:
                code = unlock.main()
            except SystemExit as exited:
                code = exited.code
        return code, execute.call_args, args

    def test_both_commands_reuse_existing_dialout(self):
        for action in ('--check', '--unlock'):
            with self.subTest(action=action):
                code, call, args = self.run_denied_action(action)
                self.assertEqual(code, 0)
                self.assertEqual(call.args[:4], ('sg', 'sg', 'dialout', '-c'))
                self.assertEqual(shlex.split(call.args[4]), ['exec', sys.executable, '-s', *args])

    def test_other_permission_errors_do_not_retry(self):
        for groups in (
            {'device_group': 30},
            {'active_group': 20},
            {'account_groups': (1000,)},
        ):
            with self.subTest(groups=groups):
                code, call, _ = self.run_denied_action('--check', **groups)
                self.assertEqual(code, 3)
                self.assertIsNone(call)


class PoweredUnlockTests(unittest.TestCase):
    def test_motor_on_unlocks_only_eligible_faults(self):
        import time
        from unittest.mock import Mock

        for mask in (0x10, 0x12):
            with self.subTest(mask=mask):
                initial = dict(flags=0x3e, motor_enabled=True, motor_power_present=True,
                               fault_latched=True, pwm_ticks=[0]*4)
                car = Mock(nonce=123, state=initial.copy(), body={'status': mask << 8 | 0xc7},
                           last_rx=time.monotonic(), last_body_rx=time.monotonic())
                car.read_state.return_value = initial

                def clear(*_args):
                    car.state.update(flags=0x3a, fault_latched=False)
                    car.body['status'] = 0x47

                car.send.side_effect = clear

                def wait(predicate, _timeout):
                    for _ in range(3):
                        if predicate():
                            return
                    raise TimeoutError('test wait expired')

                car.wait.side_effect = wait
                with patch.object(unlock, 'Chassis', return_value=car), patch.object(unlock, 'save_log'), \
                     patch.object(unlock, 'call_owner', return_value=None), \
                     patch.object(sys, 'argv', ['unlock.py', '--unlock']):
                    self.assertEqual(unlock.main(), 0 if mask == 0x10 else 3)
                self.assertEqual(car.send.call_count, int(mask == 0x10))

if __name__=='__main__':unittest.main()
