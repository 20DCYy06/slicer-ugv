"""Resolve vision source isolation using Noetic's loader; start no nodes."""
from pathlib import Path
import unittest
import roslaunch

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

    def test_vision_override_isolates_planner_and_keeps_costmap_guard_mux(self):
        baseline = load()
        vision = load('planner_raw_topic:=/cmd_vel/nav_plan_unused')
        planner = next(n for n in vision.nodes if n.type == 'move_base')
        self.assertEqual(dict(planner.remap_args)['cmd_vel'], '/cmd_vel/nav_plan_unused')
        self.assertEqual(vision.params['/slicer_nav_arc_guard/input_topic'].value,
                         '/cmd_vel/nav_raw')
        self.assertEqual({k:p.value for k,p in baseline.params.items()},
                         {k:p.value for k,p in vision.params.items()})
        types = [n.type for n in vision.nodes]
        for name in ('move_base', 'arc_velocity_guard.py', 'velocity_mux.py'):
            self.assertEqual(types.count(name), 1)
        for name in ('c30d_driver_node.py', 'control.py', 'goal.py'):
            self.assertNotIn(name, types)


if __name__ == '__main__':
    unittest.main()
