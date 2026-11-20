#!/usr/bin/env python3
"""Check active FKIE configuration without a ROS master."""
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

import yaml


ROOT = Path(__file__).resolve().parents[1]


class AirGroundConfigurationTests(unittest.TestCase):
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
