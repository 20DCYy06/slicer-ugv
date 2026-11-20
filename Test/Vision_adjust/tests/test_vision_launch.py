"""Resolve vision source isolation using Noetic's loader; start no nodes."""
from pathlib import Path
import importlib.util
import unittest
from unittest.mock import patch
import roslaunch
from rosgraph.names import resolve_name

ROOT = Path(__file__).resolve().parents[3]
LAUNCH = ROOT / 'src/Online_Navigation_with_RTK/src/ugv_bringup/launch/slicer_odom.launch'


def load(*arguments):
    config = roslaunch.config.ROSLaunchConfig()
    roslaunch.xmlloader.XmlLoader().load(str(LAUNCH), config,
        argv=['connect_control:=true'] + list(arguments), verbose=False)
    if config.config_errors:
        raise AssertionError(config.config_errors)
    return config


class VisionLaunchTests(unittest.TestCase):
    def test_default_still_routes_planner_to_guard(self):
        cfg = load()
        planner = next(n for n in cfg.nodes if n.type == 'move_base')
        self.assertEqual(dict(planner.remap_args)['cmd_vel'], '/cmd_vel/nav_raw')
        self.assertEqual(cfg.params['/slicer_nav_arc_guard/input_topic'].value,
                         '/cmd_vel/nav_raw')
        self.assertFalse(cfg.params['/slicer_nav_arc_guard/external_reverse'].value)
        self.assertEqual(cfg.params['/slicer_nav_arc_guard/planner_output_topic'].value,
                         '/cmd_vel/nav_raw')

    def test_vision_override_isolates_planner_and_keeps_costmap_guard_mux(self):
        baseline = load()
        vision = load('planner_raw_topic:=/cmd_vel/nav_plan_unused')
        planner = next(n for n in vision.nodes if n.type == 'move_base')
        self.assertEqual(dict(planner.remap_args)['cmd_vel'], '/cmd_vel/nav_plan_unused')
        self.assertEqual(vision.params['/slicer_nav_arc_guard/input_topic'].value,
                         '/cmd_vel/nav_raw')
        changed = {k for k in baseline.params
                   if baseline.params[k].value != vision.params[k].value}
        self.assertEqual(changed, {'/slicer_nav_arc_guard/planner_output_topic'})
        types = [n.type for n in vision.nodes]
        for name in ('move_base', 'arc_velocity_guard.py', 'velocity_mux.py'):
            self.assertEqual(types.count(name), 1)
        for name in ('c30d_driver_node.py', 'control.py', 'goal.py'):
            self.assertNotIn(name, types)

    def test_visual_reverse_opt_in_preserves_dwa_and_guard_routes(self):
        cfg = load('planner_raw_topic:=/cmd_vel/nav_plan_unused', 'visual_reverse:=true')
        self.assertTrue(cfg.params['/slicer_nav_arc_guard/external_reverse'].value)
        self.assertEqual(cfg.params['/slicer_nav_arc_guard/planner_output_topic'].value,
                         '/cmd_vel/nav_plan_unused')
        self.assertEqual(cfg.params['/slicer_nav_arc_guard/input_topic'].value,
                         '/cmd_vel/nav_raw')
        self.assertEqual(cfg.params['/ugv/move_base/DWAPlannerROS/min_vel_x'].value, 0.)

    def test_guard_rejects_nonboolean_opt_in_and_shared_resolved_planner_topic(self):
        path = ROOT / 'src/Online_Navigation_with_RTK/src/ugv_slicer_control/scripts/arc_velocity_guard.py'
        spec = importlib.util.spec_from_file_location('vision_test_guard', path)
        guard = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(guard)
        for parameters, message in (
                ({'external_reverse': 'true'}, 'boolean'),
                ({'external_reverse': True, 'input_topic': '/cmd_vel/nav_raw',
                  'planner_output_topic': 'cmd_vel/nav_raw'}, 'planner'),
                ({'external_reverse': True, 'input_topic': '/ugv/nav_raw',
                  'planner_output_topic': 'nav_raw'}, 'absolute'),
                ({'external_reverse': True, 'input_topic': 'nav_raw',
                  'planner_output_topic': '/cmd_vel/nav_plan_unused'}, 'absolute'),
                ({'external_reverse': True, 'input_topic': '/cmd_vel/nav_raw'}, 'planner')):
            with self.subTest(parameters=parameters), \
                    patch.object(guard.rospy, 'init_node'), \
                    patch.object(guard.rospy, 'get_param', return_value=parameters), \
                    patch.object(guard.rospy, 'resolve_name',
                                 side_effect=lambda name: resolve_name(name, '/slicer_nav_arc_guard')):
                with self.assertRaisesRegex(ValueError, message):
                    guard.main()


if __name__ == '__main__':
    unittest.main()
