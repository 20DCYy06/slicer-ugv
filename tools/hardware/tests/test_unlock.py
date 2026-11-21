import errno, shlex, stat, sys, unittest
from contextlib import ExitStack
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[3]/'Hardware/Unlock'))
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
        with patch.object(unlock,'Chassis',Fake),patch.object(unlock,'save_log'),patch.object(sys,'argv',['unlock.py',arg]):return unlock.main()
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
            patch.object(unlock, 'load_config'),
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

if __name__=='__main__':unittest.main()
