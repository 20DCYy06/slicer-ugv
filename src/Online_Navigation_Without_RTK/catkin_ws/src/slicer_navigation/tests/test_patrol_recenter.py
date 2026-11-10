#!/usr/bin/env python3
"""离线状态机回归：自转前位置跳变与自转位移保护不允许盲转。"""
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from multi_goal_patrol import Patrol


class FinalSpinRecenterTests(unittest.TestCase):
    def patrol(self):
        node = Patrol.__new__(Patrol)
        node.p = {'final_spin_reposition_attempts': 1, 'phase_settle_s': 0,
                  'wait_for_localization': True}
        node.xy = 0.15
        node.localized = True
        node.spin_failure = None
        node.chassis_ready = lambda: True
        node.running = lambda: True
        node.pause = lambda unused: True
        node.state = lambda unused: None
        node.within_tolerance = lambda unused: True
        return node

    def test_position_shift_before_final_spin_replans_before_turning(self):
        node = self.patrol()
        events = []
        samples = iter([True, False, True])
        node.verify_position = lambda unused: next(samples)
        node.send_position_goal = lambda unused: events.append('recenter') or True
        node.spin_to_heading = lambda unused, final=False: events.append('spin') or True
        node.verify_final_pose = lambda unused: True
        self.assertTrue(node.navigate({'name': 'point_001', 'x': 0, 'y': 0, 'yaw': 0}))
        self.assertEqual(events, ['recenter', 'spin'])

    def test_translation_gate_stops_then_replans_once(self):
        node = self.patrol()
        events = []
        node.verify_position = lambda unused: True
        def spin(unused, final=False):
            events.append('spin')
            if events.count('spin') == 1:
                node.spin_failure = 'translation'
                return False
            return True
        node.spin_to_heading = spin
        node.pose_error = lambda unused: (0.296, (0, 0, 0), 0)
        node.send_position_goal = lambda unused: events.append('recenter') or True
        node.verify_final_pose = lambda unused: True
        self.assertTrue(node.navigate({'name': 'point_001', 'x': 0, 'y': 0, 'yaw': 0}))
        self.assertEqual(events, ['spin', 'recenter', 'spin'])

    def test_encoder_only_limit_does_not_reset_budget_and_spin_again(self):
        node = self.patrol()
        events = []
        node.verify_position = lambda unused: True
        def spin(unused, final=False):
            events.append('spin')
            node.spin_failure = 'translation'
            return False
        node.spin_to_heading = spin
        node.pose_error = lambda unused: (0.05, (0, 0, 0), 0)
        node.send_position_goal = lambda unused: events.append('recenter') or True
        self.assertFalse(node.navigate({'name': 'point_001', 'x': 0, 'y': 0, 'yaw': 0}))
        self.assertEqual(events, ['spin'])


if __name__ == '__main__':
    unittest.main()
