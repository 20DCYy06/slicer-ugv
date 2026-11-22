import os
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from workflow_discovery import discover, master_local, process_snapshot


class Master:
    def __enter__(self):
        return self

    def __exit__(self, *_error):
        pass

    def getSystemState(self, _caller):
        return 1, '', [
            [['/scan', ['/pointcloud_to_laserscan']], ['/air_ground/v2/state', ['/ag_ugv_task']]],
            [['/ugv/move_base/goal', ['/ugv/move_base']]],
            [['/ugv/move_base/make_plan', ['/ugv/move_base']]],
        ]

    def getParam(self, _caller, key):
        assert key == '/ugv/move_base/global_costmap/global_frame'
        return 1, '', 'map'


class DiscoveryTests(unittest.TestCase):
    @patch('workflow_discovery.process_snapshot', return_value={'vision': {32}})
    @patch('workflow_discovery.ServerProxy', return_value=Master())
    def test_combines_global_ros_graph_with_non_ros_vision_process(self, server, _processes):
        graph = discover({'ROS_MASTER_URI': 'http://jetson:11311'})
        self.assertTrue(graph.online)
        self.assertEqual(graph.frame, 'map')
        self.assertEqual(graph.air_mode, 'v2')
        self.assertEqual(graph.nodes, {'/pointcloud_to_laserscan', '/ag_ugv_task', '/ugv/move_base'})
        self.assertEqual(graph.publishers['/scan'], {'/pointcloud_to_laserscan'})
        self.assertEqual(graph.subscribers['/ugv/move_base/goal'], {'/ugv/move_base'})
        self.assertEqual(graph.processes, {'vision': {32}})
        self.assertEqual(server.call_args.args[0], 'http://jetson:11311')

    @patch('workflow_discovery.process_snapshot', return_value={'vision': {32}})
    @patch('workflow_discovery.ServerProxy', side_effect=TimeoutError)
    def test_master_timeout_retains_local_processes(self, _server, _processes):
        graph = discover({})
        self.assertFalse(graph.online)
        self.assertEqual(graph.processes, {'vision': {32}})

    @patch('workflow_discovery.os.getpgid', side_effect=lambda pid: pid + 10)
    def test_process_groups_from_proc_and_exact_argv(self, _process_group):
        cases = (
            ('roslaunch ugv_bringup slicer_field_sensors.launch', ('sensors',)),
            ('roslaunch ugv_bringup slicer_body_driver.launch', ('driver',)),
            ('roslaunch ugv_bringup slicer_field_navigation.launch geographic:=false', ('odom', 'nav')),
            ('roslaunch ugv_bringup slicer_field_navigation.launch', ('map', 'nav')),
            ('/usr/bin/python3 /install/lib/ugv_navigation/geographic.py initialize', ('calibrate',)),
            ('bash /opt/ros/noetic/bin/rosrun ugv_navigation geographic.py initialize', ('calibrate',)),
            ('/usr/bin/python3 -s -m slicer_visual_grasp.app --run', ('vision',)),
            ('bash /opt/ros/noetic/bin/rosrun slicer_visual_grasp run.sh --run', ('vision',)),
            ('/usr/bin/python3 -s -m slicer_visual_grasp.app', ('vision',)),
            ('roslaunch ugv_bringup slicer_air_ground.launch', ('air', 'v1')),
            ('roslaunch ag_ugv_task ugv.launch monitor_only:=true', ('air', 'v2')),
            ('/usr/bin/python3 /install/lib/ag_ugv_task/ag_ugv_task_node.py', ('air', 'v2')),
            ('/usr/bin/python3 -s -m slicer_visual_grasp.app --check', ()),
            ('echo slicer_field_navigation.launch', ()),
        )
        expected = {}
        with tempfile.TemporaryDirectory() as root:
            for pid, (command, keys) in enumerate(cases, 100):
                proc = Path(root) / str(pid)
                proc.mkdir()
                (proc / 'cmdline').write_bytes(b'\x00'.join(arg.encode() for arg in command.split()) + b'\x00')
                for key in keys:
                    expected.setdefault(key, set()).add(pid + 10)
            self.assertEqual(process_snapshot(Path(root), uid=os.geteuid()), expected)

    def test_bootstrap_is_permitted_only_for_local_master_address(self):
        self.assertTrue(master_local({'ROS_MASTER_URI': 'http://127.0.0.1:11319'}))
        self.assertFalse(master_local({'ROS_MASTER_URI': 'http://192.0.2.44:11311'}))


if __name__ == '__main__':
    unittest.main()
