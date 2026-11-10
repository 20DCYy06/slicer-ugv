#!/usr/bin/env python3
"""隔离Docker验收：真实SLAM/规划器，合成扫描与假底盘；没有设备映射。"""
import ast,os,signal,subprocess,sys,tempfile,time,unittest,xmlrpc.client
from pathlib import Path
import xml.etree.ElementTree as ET
import yaml
PACKAGE=Path(__file__).resolve().parents[1];ROOT=PACKAGE.parents[2]
sys.path.insert(0,str(PACKAGE/'scripts'))
from prepare_runtime import generate
class MigrationChecks(unittest.TestCase):
    def test_manifests(self):
        from catkin_pkg.package import parse_package
        for p in (ROOT/'catkin_ws/src').glob('*/package.xml'):parse_package(str(p)).validate()
    def test_all_runtime_executables_resolve(self):
        import roslib.packages
        for package,node in [('livox_ros_driver2','livox_ros_driver2_node'),('fast_lio','fastlio_mapping'),('slicer_navigation','arc_velocity_guard.py'),('slicer_wasd_control','c30d_driver_node.py'),('slicer_navigation_without_mapping','goal_display.py'),('slicer_navigation_without_mapping','reset_map.py')]:
            self.assertTrue(roslib.packages.find_node(package,node),(package,node))
    def test_lidar_only_and_online_map(self):
        x=ET.parse(PACKAGE/'launch/slicer_nav_without_mapping.launch');text=ET.tostring(x.getroot(),encoding='unicode')
        self.assertNotIn('map_server',text);self.assertNotIn('amcl',text);self.assertNotIn('camera',text);self.assertIn('gmapping',text)
        c=yaml.safe_load((PACKAGE/'config/costmap_params.yaml').read_text())
        for key in ['global_costmap','local_costmap']:
            names=[p['name'] for p in c[key]['plugins']]
            self.assertIn('obstacle_layer',names);self.assertEqual(names.count('inflation_layer'),1);self.assertNotIn('depth_layer',names)
    def test_latest_geometry_and_generated_control(self):
        source=ROOT/'config/slicer_config.yaml';before=source.read_bytes()
        with tempfile.TemporaryDirectory() as d:
            nav=generate(ROOT,d);c=yaml.safe_load(source.read_text());ch=c['chassis']
            self.assertAlmostEqual(ch['wheel_diameter'],.065);self.assertAlmostEqual(ch['wheelbase'],.13);self.assertAlmostEqual(ch['track_width'],.179);self.assertAlmostEqual(ch['body_width'],.206)
            self.assertEqual(c['sensors']['base_to_laser']['xyz'],[.065,0.,.095])
            scan=yaml.safe_load((Path(d)/'.generated/scan.yaml').read_text());self.assertEqual(scan['min_height'],.05)
            control=yaml.safe_load((Path(d)/'config/control.yaml').read_text());host=yaml.safe_load(Path('/workspace2/Hardware/config.yaml').read_text())
            self.assertEqual(control['serial_number'],host['serial']['serial_number']);self.assertEqual(control['encoder_signs'],host['serial']['encoder_signs']);self.assertEqual(control['control_mode'],'body_speed')
            self.assertEqual(nav['TebLocalPlannerROS']['min_turning_radius'],0.)
            self.assertEqual(nav['TebLocalPlannerROS']['max_vel_x'],c['navigation']['max_forward'])
            self.assertEqual(nav['global_costmap']['footprint'],nav['local_costmap']['footprint'])
        self.assertEqual(source.read_bytes(),before)
    def test_python_syntax(self):
        for folder in ['slicer_navigation','slicer_navigation_without_mapping','slicer_wasd_control','slicer_bringup']:
            for p in (ROOT/'catkin_ws/src'/folder).rglob('*.py'):ast.parse(p.read_text(),filename=str(p))
    def test_no_old_workspace_mounted(self):
        mounts=Path('/proc/mounts').read_text();self.assertNotIn('/home/potato/Slicer_Workspace/',mounts)
if __name__=='__main__':
    result=unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromTestCase(MigrationChecks))
    if not result.wasSuccessful():raise SystemExit(1)
    for name in ['check_driver.py','check_patrol.py','check_goal_display.py']:
        subprocess.run([sys.executable,str(PACKAGE/'tests'/name)],check=True,env=dict(os.environ,QT_QPA_PLATFORM='offscreen'))
    subprocess.run(['rosrun','slicer_navigation','slicer_bspline_check'],check=True)
    subprocess.run([sys.executable,str(ROOT/'catkin_ws/src/slicer_navigation/tests/test_arm_cloud_filter.py')],check=True)
    with tempfile.TemporaryDirectory() as d:
        generate(ROOT,d);stream=open('/tmp/online-master.log','w');master=subprocess.Popen(['roscore'],stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
        try:
            for _ in range(100):
                try:
                    if xmlrpc.client.ServerProxy('http://127.0.0.1:11311').getPid('/test')[0]==1:break
                except OSError:time.sleep(.1)
            else:raise RuntimeError('Isolated ROS master did not start')
            run=subprocess.run([sys.executable,str(PACKAGE/'tests/integration.py'),d],timeout=110)
            if run.returncode:
                for name in ['online-launch','online-mock']:
                    p=Path('/tmp/'+name+'.log')
                    if p.exists():print(p.read_text()[-14000:])
                raise SystemExit(run.returncode)
        finally:
            if master.poll() is None:
                os.killpg(master.pid,signal.SIGINT)
                try:master.wait(timeout=8)
                except subprocess.TimeoutExpired:os.killpg(master.pid,signal.SIGKILL);master.wait()
            stream.close()
    print('WORKSPACE2_ONLINE_NAV_CHECK_PASS: no real devices/network/hardware motion',flush=True)
