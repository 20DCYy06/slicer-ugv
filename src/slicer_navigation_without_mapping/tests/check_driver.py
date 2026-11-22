#!/usr/bin/env python3
"""新ROS适配层离线测试：假MCU，绝不连接真实串口。"""
import copy,math,os,struct,sys,unittest
from pathlib import Path
from unittest.mock import patch
import yaml
ROOT=Path(__file__).resolve().parents[3];SCRIPTS=ROOT/'src/slicer_wasd_control/scripts'
WORKSPACE=Path(os.environ.get('WORKSPACE2_ROOT') or ROOT)
sys.path[:0] = [str(SCRIPTS), str(ROOT / 'tools')]
from hardware_path import hardware_root

HARDWARE = hardware_root(WORKSPACE)
sys.path.insert(0, str(HARDWARE / 'Host'))
from driver_core import Session,Odometry,EncoderTap
from chassis import Chassis
from protocol import packet,Parser

class Fake:
    def __init__(self):
        self.state=dict(flags=0x3a,fault_latched=False,configured=True,motor_enabled=True,motor_power_present=True)
        self.last_rx=10.;self.body={'imu_flags':7};self.replies={};self.sent=[];self.sessions=0
    def poll(self,*args):pass
    def speed(self,v,w):self.sent.append((v,w))
    def connect(self):
        assert not self.state['motor_enabled'];self.sessions+=1
        self.state.update(fault_latched=False,flags=0x2a);self.body={'imu_flags':7}

class DriverChecks(unittest.TestCase):
    def config(self):return dict(command_timeout_s=.25,telemetry_timeout_s=1.,max_linear=.6,max_angular=2.4725146199,wheel_diameter=.065,track_width=.179,wheelbase=.13,counts_per_revolution=60000,encoder_signs=[1,-1,1,-1])
    def test_actual_ros_driver_import_without_opening_serial(self):
        import importlib.util
        spec=importlib.util.spec_from_file_location('workspace2_driver',SCRIPTS/'c30d_driver_node.py')
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        self.assertEqual(module.C30D_TOOLS, HARDWARE)
    def test_required_fuser_runtime_tool_is_available(self):
        import shutil,subprocess
        self.assertIsNotNone(shutil.which('fuser'),'需要安装 psmisc')
        self.assertEqual(subprocess.run(['fuser','--version'],capture_output=True).returncode,0)
    def test_actual_device_selection_checks_free_and_busy_ports_without_opening_serial(self):
        import chassis,tempfile
        expected=('1a86','55d4','5CF0016129')
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'fake_port';path.write_bytes(b'')
            cfg={'serial':dict(port=str(path),vid=expected[0],pid=expected[1],serial_number=expected[2])}
            with patch.object(chassis,'identity',return_value=expected):
                self.assertEqual(chassis.device(cfg),(str(path),expected))
                with path.open('rb'):
                    with self.assertRaisesRegex(RuntimeError,'串口被占用'):chassis.device(cfg)
    def test_actual_chassis_sends_twist_only_not_pwm(self):
        car=Chassis.__new__(Chassis);car.nonce=123
        car.config={'speed':{'linear_max':.6,'angular_max':2.5}};sent=[];car.send=lambda k,p:sent.append((k,p))
        car.speed(.4,-.7)
        self.assertEqual(sent,[(0x16,struct.pack('<Ihh',123,400,-700))])
        self.assertNotIn(0x11,[k for k,p in sent])
    def test_connect_waits_for_wheel_telemetry_after_imu(self):
        import time
        from chassis import load_config
        car=Chassis.__new__(Chassis)
        car.config=load_config()
        car.path='/TEST_ONLY_DELAYED_0X90'
        car.nonce=123
        car.replies={}
        car.body=car.state=None
        car.last_rx=0.
        replies={
            1:(0x81,struct.pack('<4sBBHHI',b'SLC2',2,4,60000,500,car.nonce)),
            0x12:(0x82,struct.pack('<IB',car.nonce,1)),
            0x17:(0x84,struct.pack('<IBBBB',car.nonce,1,0xea,7,0)),
            0x18:(0x85,struct.pack('<IBHHH',car.nonce,100,982,1600,1571)),
        }
        car.send=lambda kind,payload: car.replies.update([replies[kind]])

        def poll(timeout):
            if car.body is None:
                car.body={'imu_flags':7}
            elif car.state is None:
                car.state={'fault_latched':False}
                car.last_rx=time.monotonic()

        car.poll=poll
        car.connect()
        self.assertIsNotNone(car.state)
        self.assertGreater(car.last_rx,0.)
    def test_forward_curve_pure_spin_reverse_retained(self):
        car=Fake();session=Session(car,self.config())
        for v,w in [(.4,.6),(0.,1.2),(-.2,0.)]:
            session.command(v,w,10.);out=session.cycle(10.);self.assertEqual(out[:2],(v,w));self.assertTrue(out[2])
    def test_stale_command_zero(self):
        car=Fake();session=Session(car,self.config());session.command(.4,.5,9.)
        self.assertEqual(session.cycle(10.)[:2],(0.,0.))
    def test_fault_on_never_rehandshakes_and_off_drops_old_commands(self):
        car=Fake();session=Session(car,self.config());car.state.update(fault_latched=True,flags=0x3e)
        session.command(.6,1.,10.);self.assertEqual(session.cycle(10.)[:2],(0.,0.));self.assertEqual(car.sessions,0)
        car.state.update(motor_enabled=False,flags=0x2e);session.command(.6,1.,10.)
        out=session.cycle(10.);self.assertEqual(out[:2],(0.,0.));self.assertTrue(out[4]);self.assertEqual(car.sessions,1)
        car.state['motor_enabled']=True;session.command(.3,.2,10.1)
        self.assertEqual(session.cycle(10.1)[:2],(.3,.2))
    def test_failed_recovery_stays_zero_without_process_exit(self):
        car=Fake();session=Session(car,self.config());car.state.update(fault_latched=True,motor_enabled=False,flags=0x2e)
        car.connect=lambda:(_ for _ in ()).throw(TimeoutError('ACK missing'))
        out=session.cycle(10.);self.assertEqual(out[:2],(0.,0.));self.assertFalse(out[2]);self.assertIn('ACK missing',session.recovery_error)
    def test_powered_link_lock_recovers_in_place_and_requires_a_new_command(self):
        car=Fake();car.nonce=123;car.last_body_rx=10.
        frames=[];car.send=lambda kind,payload:frames.append((kind,payload))
        session=Session(car,self.config())
        car.body['status']=0x10c7
        car.state.update(fault_latched=True,flags=0x3e)
        session.command(.4,.5,10.)
        self.assertEqual(session.cycle(10.)[:3],(0.,0.,False))
        self.assertEqual(frames,[(0x19,struct.pack('<IB',123,0))])
        car.state.update(fault_latched=False,flags=0x3a);car.body['status']=0x47
        self.assertEqual(session.cycle(10.05),(0.,0.,False,False,True))
        self.assertEqual(session.cycle(10.1)[:3],(0.,0.,True))
        session.command(.3,.2,10.15)
        self.assertEqual(session.cycle(10.15)[:3],(.3,.2,True))
        self.assertEqual(car.sessions,0)
    def test_telemetry_loss_stops_with_error(self):
        car=Fake();session=Session(car,self.config())
        with self.assertRaisesRegex(RuntimeError,'telemetry timeout'):session.cycle(12.)
    def test_recovery_capable_telemetry_gap_holds_until_new_body_sample(self):
        car=Fake();car.nonce=123;car.last_body_rx=10.
        car.send=lambda *_:None
        car.body['status']=0x47
        session=Session(car,dict(self.config(),telemetry_timeout_s=.25))
        session.command(.4,.5,10.26)
        self.assertEqual(session.cycle(10.26)[:3],(0.,0.,False))
        self.assertEqual(car.sent,[])
        car.last_rx=10.27
        self.assertEqual(session.cycle(10.27)[:3],(0.,0.,False))
        car.last_body_rx=10.28
        self.assertEqual(session.cycle(10.28),(0.,0.,False,False,True))
        session.command(.4,.5,10.29)
        self.assertEqual(session.cycle(10.29)[:3],(.4,.5,True))
        with self.assertRaisesRegex(RuntimeError,'telemetry timeout'):session.cycle(14.)
    def test_new_protective_fault_is_not_cleared_by_navigation_in_motor_off(self):
        car=Fake();car.nonce=123;car.last_body_rx=10.
        car.body['status']=0x0246
        car.state.update(fault_latched=True,motor_enabled=False,flags=0x2e)
        session=Session(car,self.config())
        self.assertEqual(session.cycle(10.)[:3],(0.,0.,False))
        self.assertEqual(car.sessions,0)
    def test_encoder_distance_and_imu_yaw_instead_of_command(self):
        c=self.config();odom=Odometry(c);self.assertIsNone(odom.update((0,[0]*4),{'imu_flags':7,'wz':0.}))
        value=odom.update((100000,[s*6000 for s in c['encoder_signs']]),{'imu_flags':7,'wz':.5})
        self.assertAlmostEqual(value[3],math.pi*.065,places=8);self.assertAlmostEqual(value[2],.05);self.assertEqual(value[4],.5)
    def test_rear_axle_mapping_keeps_positive_encoder_yaw_without_imu(self):
        c=self.config()
        c['wheel_ports']=dict(left_front='D',left_rear='A',right_front='C',right_rear='B')
        c['encoder_signs']=[-1,1,1,-1]
        odom=Odometry(c)
        self.assertIsNone(odom.update((0,[0]*4),None))
        result=odom.update((100000,[6000]*4),None)
        self.assertAlmostEqual(result[3],0.)
        expected_yaw_rate=2*math.pi*.065*.1/(.179+.13)/.1
        self.assertAlmostEqual(result[4],expected_yaw_rate)
    def test_new_board_identity_and_verified_polarities(self):
        c=yaml.safe_load((HARDWARE / 'config.yaml').read_text())
        self.assertEqual((c['serial']['port'],c['serial']['serial_number']),('/dev/c30d','5CF0016377'))
        self.assertEqual(c['serial']['encoder_signs'],[-1,1,1,-1])
        self.assertEqual(c['serial']['pwm_signs'],[-1,1,-1,-1])
if __name__=='__main__':unittest.main(verbosity=2)
