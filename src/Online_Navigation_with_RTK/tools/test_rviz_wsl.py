"""Observer argv/preset checks only; fake Docker/ip, no ROS, GUI or network."""
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest

import yaml


ROOT = Path(__file__).resolve().parents[1]


class RvizObserverTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix='ugv-rviz-argv-')
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        (self.root / 'tools').mkdir()
        self.script = self.root / 'tools/rviz_wsl.sh'
        shutil.copy2(ROOT / 'tools/rviz_wsl.sh', self.script)
        config = self.root / 'src/ugv_bringup/config'
        config.mkdir(parents=True)
        (config / 'rviz_observer.env').write_text('UGV_RVIZ_MASTER_URI=http://192.168.3.68:11311\n')
        (config / 'slicer_runtime.env').write_text(
            'UGV_RUNTIME_IMAGE=DO_NOT_USE_ARM\nUGV_RUNTIME_IP=192.168.3.68\n')
        self.bin = self.root / 'bin'
        self.bin.mkdir()
        docker = self.bin / 'docker'
        docker.write_text('#!' + sys.executable + '\nimport json,os,sys\n'
                          'open(os.environ["ARGV"],"w").write(json.dumps(sys.argv[1:]))\n')
        docker.chmod(0o755)
        ip = self.bin / 'ip'
        ip.write_text('#!' + sys.executable + '\nimport json,os,sys\n'
                      'open(os.environ["IP_ARGV"],"w").write(json.dumps(sys.argv[1:]))\n'
                      'print("192.168.3.68 dev TEST src 192.168.3.208 uid 1000")\n')
        ip.chmod(0o755)
        self.env = dict(os.environ)
        for name in ('ROS_MASTER_URI', 'ROS_IP', 'ROS_HOSTNAME', 'UGV_NOETIC_IMAGE',
                     'UGV_RVIZ_MASTER_URI'):
            self.env.pop(name, None)
        self.env.update(PATH=str(self.bin) + os.pathsep + self.env['PATH'],
                        ARGV=str(self.root / 'docker.json'), IP_ARGV=str(self.root / 'ip.json'))

    def run_observer(self, mode, *extra):
        result = subprocess.run(['bash', str(self.script), mode, *extra], env=self.env,
                                stdin=subprocess.DEVNULL, capture_output=True,
                                text=True, timeout=10)
        self.assertEqual(result.returncode, 0, result.stderr)
        return json.loads((self.root / 'docker.json').read_text())

    def run_interactive_observer(self, answer):
        master, slave = os.openpty()
        try:
            process = subprocess.Popen(['bash', str(self.script), 'online'], env=self.env,
                                       stdin=slave, stdout=subprocess.PIPE,
                                       stderr=subprocess.PIPE, text=True)
            os.close(slave)
            slave = None
            os.write(master, (answer+'\n').encode())
            stdout, stderr = process.communicate(timeout=10)
            self.assertEqual(process.returncode, 0, stdout+stderr)
            self.assertIn('ROS master', stderr)
            return json.loads((self.root / 'docker.json').read_text())
        finally:
            os.close(master)
            if slave is not None:
                os.close(slave)

    def test_online_uses_observer_config_and_routes_its_own_ip(self):
        args = self.run_observer('online')
        self.assertIn('ROS_MASTER_URI=http://192.168.3.68:11311', args)
        self.assertIn('ROS_IP=192.168.3.208', args)
        self.assertEqual(args[-1], '/rviz/online_navigation.rviz')
        self.assertEqual(json.loads((self.root / 'ip.json').read_text()),
                         ['-4', 'route', 'get', '192.168.3.68'])
        self.assertIn('industrial-cup/nav-slicer:20261023-dwa', args)
        self.assertNotIn('DO_NOT_USE_ARM', args)
        self.assertNotIn('--device', args)

    def test_external_ros_values_override_file_without_route_query(self):
        self.env.update(ROS_MASTER_URI='http://192.168.1.9:11311', ROS_IP='192.168.1.8')
        args = self.run_observer('online')
        self.assertIn('ROS_MASTER_URI=http://192.168.1.9:11311', args)
        self.assertIn('ROS_IP=192.168.1.8', args)
        self.assertFalse((self.root / 'ip.json').exists())

    def test_explicit_master_ip_overrides_environment_and_selects_its_route(self):
        self.env['ROS_MASTER_URI'] = 'http://192.168.1.9:11311'
        args = self.run_observer('online', '192.168.10.33')
        self.assertIn('ROS_MASTER_URI=http://192.168.10.33:11311', args)
        self.assertEqual(json.loads((self.root / 'ip.json').read_text()),
                         ['-4', 'route', 'get', '192.168.10.33'])

    def test_explicit_master_uri_preserves_custom_port(self):
        args = self.run_observer('online', 'http://192.168.10.33:11321')
        self.assertIn('ROS_MASTER_URI=http://192.168.10.33:11321', args)
        self.assertEqual(json.loads((self.root / 'ip.json').read_text()),
                         ['-4', 'route', 'get', '192.168.10.33'])

    def test_terminal_input_selects_master_for_this_run(self):
        args = self.run_interactive_observer('192.168.10.33')
        self.assertIn('ROS_MASTER_URI=http://192.168.10.33:11311', args)

    def test_terminal_enter_keeps_configured_default(self):
        args = self.run_interactive_observer('')
        self.assertIn('ROS_MASTER_URI=http://192.168.3.68:11311', args)

    def test_extra_arguments_do_not_start_docker(self):
        result = subprocess.run(['bash', str(self.script), 'online',
                                 '192.168.10.33', 'extra'], env=self.env,
                                stdin=subprocess.DEVNULL, capture_output=True,
                                text=True, timeout=10)
        self.assertEqual(result.returncode, 2)
        self.assertFalse((self.root / 'docker.json').exists())

    def test_existing_presets_keep_their_names(self):
        for mode, preset in (('perception', 'small_obstacle.rviz'),
                             ('navigation', 'odom_navigation.rviz')):
            with self.subTest(mode=mode):
                self.assertEqual(self.run_observer(mode)[-1], '/rviz/' + preset)

    def test_online_preset_is_read_only_and_contains_map_depth_and_navigation(self):
        config = yaml.safe_load((ROOT / 'src/ugv_bringup/rviz/online_navigation.rviz').read_text())
        manager = config['Visualization Manager']
        self.assertEqual(manager['Global Options']['Fixed Frame'], 'map')
        displays = {item.get('Topic'): item for item in manager['Displays']}
        for topic in ('/map', '/scan', '/odom', '/ugv/move_base/GlobalPlanner/plan',
                      '/ugv/move_base/DWAPlannerROS/local_plan',
                      '/ugv/move_base/local_costmap/footprint',
                      '/ugv/move_base/global_costmap/costmap',
                      '/ugv/move_base/local_costmap/costmap', '/online_depth/obstacles'):
            self.assertTrue(displays[topic]['Enabled'], topic)
        self.assertFalse(displays['/online_depth/clearing']['Enabled'])
        self.assertEqual([tool['Class'] for tool in manager['Tools']],
                         ['rviz/Interact', 'rviz/MoveCamera', 'rviz/Select',
                          'rviz/FocusCamera', 'rviz/Measure'])

    def test_observer_template_contains_only_the_master_setting(self):
        lines = (ROOT / 'src/ugv_bringup/config/rviz_observer.env').read_text().splitlines()
        values = [line for line in lines if line and not line.startswith('#')]
        self.assertEqual(len(values), 1)
        name, master = values[0].split('=', 1)
        self.assertEqual(name, 'UGV_RVIZ_MASTER_URI')
        self.assertTrue(master.startswith('http://'), master)


if __name__ == '__main__':
    unittest.main()
