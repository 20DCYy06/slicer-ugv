#!/usr/bin/env python3
"""Static configuration checks only; no ROS import, master, network or devices."""
from pathlib import Path
import json
import os
import shutil
import subprocess
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET

import yaml


ROOT = Path(__file__).resolve().parent


class AirGroundConfigurationTests(unittest.TestCase):
    def command_fixture(self, mode, installed):
        """Capture Docker argv; never invoke Docker or the master probe."""
        with tempfile.TemporaryDirectory() as directory:
            workspace = Path(directory)
            communication = workspace / 'src/conmunication'
            (communication / 'config').mkdir(parents=True)
            shutil.copy2(ROOT / 'run.sh', communication / 'run.sh')
            shutil.copy2(ROOT / 'config/network.env', communication / 'config/network.env')
            install = workspace / 'src/Online_Navigation_with_RTK/install'
            if installed:
                install.mkdir(parents=True)
                (install / 'setup.bash').write_text('# fixture only\n')
            binaries = workspace / 'bin'
            binaries.mkdir()
            capture = workspace / 'docker.jsonl'
            docker = binaries / 'docker'
            docker.write_text('#!' + sys.executable + '\n'
                              'import json, os, sys\n'
                              "with open(os.environ['COMM_CAPTURE'], 'a') as stream:\n"
                              "    stream.write(json.dumps(sys.argv[1:]) + '\\n')\n"
                              "sys.exit(1 if sys.argv[1] == 'inspect' else 0)\n")
            docker.chmod(0o755)
            # start's probe exits locally; this executable performs no network IO.
            probe = binaries / 'python3'
            probe.write_text('#!/bin/sh\nexit 1\n')
            probe.chmod(0o755)
            env = dict(os.environ, PATH=str(binaries)+os.pathsep+os.environ['PATH'], COMM_CAPTURE=str(capture))
            result = subprocess.run(['bash', str(communication / 'run.sh'), mode], env=env,
                                    capture_output=True, text=True, timeout=5.)
            self.assertEqual(result.returncode, 0, result.stderr)
            calls = [json.loads(line) for line in capture.read_text().splitlines()]
            return calls, str(install)

    def test_communication_runtime_mounts_own_generated_messages_readonly_and_sources_after_ros(self):
        calls, install = self.command_fixture('start', installed=True)
        runs = [call for call in calls if call[0] == 'run']
        self.assertEqual(len(runs), 4)
        checked = set()
        for call in runs:
            name = call[call.index('--name')+1]
            if not name.endswith(('-sync', '-discovery', '-state-relay')):
                continue
            checked.add(name.rsplit('-', 1)[-1])
            self.assertIn(install+':/rtk_install:ro', call)
            script = call[call.index('-c')+1]
            self.assertIn('[[ -f /rtk_install/setup.bash ]]', script)
            self.assertIn('source /rtk_install/setup.bash --extend', script)
            self.assertLess(script.index('source /opt/ros/noetic/setup.bash'),
                            script.index('source /rtk_install/setup.bash --extend'))
        self.assertEqual(checked, {'sync', 'discovery', 'relay'})

    def test_missing_install_keeps_runtime_and_network_none_check_available(self):
        for mode in ('start', 'check'):
            calls, install = self.command_fixture(mode, installed=False)
            runs = [call for call in calls if call[0] == 'run']
            self.assertTrue(runs)
            for call in runs:
                self.assertNotIn(install+':/rtk_install:ro', call)
                if mode == 'check':
                    self.assertEqual(call[call.index('--network')+1], 'none')
                    self.assertNotIn('--device', call)

    def test_formal_interfaces_are_synced_with_declared_types_and_latching(self):
        config = yaml.safe_load((ROOT / 'config/sync_topics.yaml').read_text())
        interfaces = config.get('air_ground_interfaces', {})
        self.assertEqual(interfaces, {
            'command': {'topic': '/air_ground/command',
                        'type': 'slicer_air_ground_msgs/Command', 'latch': False},
            'state': {'topic': '/air_ground/state',
                      'type': 'std_msgs/String', 'latch': True},
        })
        self.assertTrue({'/air_ground/command', '/air_ground/state'} <= set(config['sync_topics']))
        package = ET.parse(ROOT / 'package.xml').getroot()
        self.assertIn('slicer_air_ground_msgs', [item.text for item in package.findall('exec_depend')])

    def test_legacy_receiver_is_disabled_by_default_and_explicitly_opt_in(self):
        launch = ET.parse(ROOT / 'launch/fkie_ugv_sync.launch').getroot()
        arguments = {item.attrib['name']: item.attrib.get('default') for item in launch.findall('arg')}
        self.assertEqual(arguments.get('enable_legacy_waypoints'), 'false')
        receiver = launch.find("node[@name='ugv_waypoint_receiver']")
        self.assertIsNotNone(receiver)
        self.assertEqual(receiver.attrib.get('if'), '$(arg enable_legacy_waypoints)')
        topics = yaml.safe_load((ROOT / 'config/sync_topics.yaml').read_text())['sync_topics']
        self.assertNotIn('/uav/mission/waypoints', topics)

    def test_existing_state_routes_are_preserved_without_relaying_formal_task_state(self):
        routes = yaml.safe_load((ROOT / 'config/relay.yaml').read_text())['routes']
        self.assertEqual({item['source']: item['target'] for item in routes}, {
            '/odom': '/ugv/odom', '/global/odom': '/ugv/pose',
            '/slicer_c30d_driver/ready': '/ugv/chassis_ready',
            '/slicer_multi_goal_patrol/state': '/ugv/mission/state',
        })
        sync = yaml.safe_load((ROOT / 'config/sync_topics.yaml').read_text())['sync_topics']
        self.assertTrue({'/uav/test', '/ugv/odom', '/ugv/pose',
                         '/ugv/chassis_ready', '/ugv/mission/state'} <= set(sync))
        self.assertFalse(any(item['target'] == '/air_ground/state' for item in routes))


if __name__ == '__main__':
    unittest.main()
