#!/usr/bin/env python3
"""本次迁移定向回归：不打开串口、不发布速度、不读取真实凭据。"""
import ast,json,os,struct,sys,unittest
from pathlib import Path
from unittest.mock import patch
import yaml
ROOT=Path(__file__).resolve().parents[1];SCRIPTS=ROOT/'src/ugv_slicer_control/scripts'
sys.path.insert(0,str(SCRIPTS));import c30d_link
class MigrationTests(unittest.TestCase):
    def test_new_board_identity_signs_and_geometry(self):
        c=yaml.safe_load((ROOT/'src/ugv_bringup/config/slicer_driver_body_speed.yaml').read_text())
        self.assertEqual(c['control_mode'],'body_speed');self.assertEqual(c['serial_number'],'5CF0016129')
        self.assertEqual(c['pwm_signs'],[1,-1,-1,-1]);self.assertEqual(c['encoder_signs'],[1,-1,1,-1])
        self.assertEqual((c['wheel_diameter'],c['track_width'],c['wheelbase']),(.065,.179,.13))
        self.assertEqual((c['motor_output_limit_percent'],c['mcu_pwm_accept_max_permille']),(100,800))
    def test_new_lidar_mount_and_antenna(self):
        c=yaml.safe_load((ROOT/'src/ugv_bringup/config/online_runtime/config/slicer_config.yaml').read_text())
        self.assertEqual(c['sensors']['base_to_laser']['xyz'],[.065,0.,.095]);self.assertEqual(c['sensors']['scan_min_height'],.05)
        site=yaml.safe_load((ROOT/'src/ugv_bringup/config/gnss_site.yaml').read_text());self.assertEqual(site['antenna_in_base_link_m'][:2],[.065,0.])
    def test_output_ceiling_0x18_and_reject_wrong_firmware(self):
        car=c30d_link.Chassis.__new__(c30d_link.Chassis);car.nonce=123;sent=[]
        def send(kind,payload):sent.append((kind,payload));car.output_limit_ack=(123,payload[-1],800,1600,1280*payload[-1]//100)
        car.send=send;car.wait=lambda pred,timeout:self.assertTrue(pred())
        self.assertEqual(car.set_output_limit_percent(100)['pwm_ticks'],1280)
        self.assertEqual(sent,[(0x18,struct.pack('<IB',123,100))])
        def invalid(kind,payload):car.output_limit_ack=(123,100,982,1600,1571)
        car.send=invalid
        with self.assertRaises(RuntimeError):car.set_output_limit_percent(100)
    def test_body_twist_wire_format_stays_vx_wz(self):
        car=c30d_link.Chassis.__new__(c30d_link.Chassis);car.nonce=123;sent=[];car.send=lambda kind,data:sent.append((kind,data))
        car.body_speed(.4,-.25);self.assertEqual(sent,[(0x16,struct.pack('<Ihh',123,400,-250))])
    def test_only_new_reference_in_default_wrappers(self):
        for name in ['tools/noetic.sh','tools/slicer_shell.sh']:
            s=(ROOT/name).read_text();self.assertIn('Online_Navigation_Without_RTK',s);self.assertNotIn('/home/potato/Slicer_Workspace"',s);self.assertNotIn('/devel_navigation/',s)
    def test_runtime_launch_parses_from_install_and_no_double_tf(self):
        import subprocess
        where=subprocess.check_output(['rospack','find','ugv_bringup'],text=True).strip()
        self.assertEqual(where,str(ROOT/'install/share/ugv_bringup'))
        import roslib.packages
        for package,node in [('ugv_slicer_control','velocity_mux.py'),('ugv_navigation','geographic.py')]:
            found=roslib.packages.find_node(package,node);self.assertEqual(len(found),1,found)
        for name in ['slicer_field_sensors.launch','slicer_field_navigation.launch','slicer_body_driver.launch']:
            p=subprocess.run(['roslaunch','--dump-params','ugv_bringup',name],capture_output=True,text=True)
            self.assertEqual(p.returncode,0,p.stderr)
            params=yaml.safe_load(p.stdout);self.assertNotIn('/slam_gmapping/base_frame',params)
            if name=='slicer_body_driver.launch':
                self.assertEqual(params['/slicer_c30d_driver/control_mode'],'body_speed');self.assertFalse(params['/slicer_c30d_driver/publish_tf'])
if __name__=='__main__':unittest.main(verbosity=2)
