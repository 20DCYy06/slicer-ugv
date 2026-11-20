#!/usr/bin/env python3
"""Offline checks for the simplified old-rover-style patrol contract."""
import math
from pathlib import Path
import sys
import unittest
import yaml

PACKAGE = Path(__file__).resolve().parents[1]
ROOT = PACKAGE.parents[2]
sys.path.insert(0, str(PACKAGE / 'scripts'))
from multi_goal_patrol import Patrol, angle_error
from navigation_core import saturate_twist, slew_arc, compensate_turn, is_stalled


class PatrolFlowTest(unittest.TestCase):
    def test_stale_path_is_reported_as_stale_not_empty_without_changing_recovery(self):
        patrol = Patrol.__new__(Patrol)
        patrol.p = {'stall_time': 3.}
        patrol.plan_topics = {'global': (10., 54), 'local': (14., 0)}
        self.assertEqual(patrol.plan_diagnostic('global', 15.),
                         {'poses': 54, 'age_s': 5., 'fresh': False})
        self.assertEqual(patrol.plan_count('global', 15.), 0)
        self.assertEqual(patrol.plan_diagnostic('local', 15.),
                         {'poses': 0, 'age_s': 1., 'fresh': True})
        self.assertEqual(patrol.plan_diagnostic('missing', 15.),
                         {'poses': None, 'age_s': None, 'fresh': False})

    def test_angle_wrap(self):
        self.assertAlmostEqual(angle_error(-math.pi + .1, math.pi - .1), .2)

    def test_curvature_saturation_and_pure_spin(self):
        v, w = saturate_twist(.25, 1.4, .125, .05, .7)
        self.assertAlmostEqual(v, .125)
        self.assertAlmostEqual(w, .7)
        self.assertAlmostEqual(v / w, .25 / 1.4)
        self.assertEqual(saturate_twist(0., -.3, .125, .05, .7), (0., -.3))
        self.assertEqual(saturate_twist(-.10, 0., .125, .05, .7), (-.05, 0.))
        self.assertEqual(slew_arc((.1, .2), (0., 0.), .05, .2, .45), (0., 0.))

    def test_encoder_vx_fusion_is_retained(self):
        fusion = yaml.safe_load((ROOT / '.generated/fusion.yaml').read_text())
        self.assertEqual(fusion['odom1'], '/wheel/odom')
        self.assertEqual(fusion['odom1_config'], [i == 6 for i in range(15)])
        self.assertTrue(fusion['publish_tf'])

    def test_linear_sign_jitter_does_not_interrupt_turning(self):
        v, w = slew_arc((.001, -.2), (-.001, -.2), .05, .2, 1.)
        self.assertAlmostEqual(v, -.001)
        self.assertAlmostEqual(w, -.2)
        self.assertEqual(slew_arc((v, w), (0., -.2), .05, .2, 1.), (0., -.2))
        # Reversing angular direction ramps through zero without erasing forward speed.
        v, w = slew_arc((.1, .3), (.1, -.3), .05, .2, 1.)
        self.assertAlmostEqual(v, .1)
        self.assertAlmostEqual(w, .25)

    def test_no_reverse_keeps_spin_and_applies_turn_floor(self):
        turn_floor = 200. / 283.11258278
        v, w = saturate_twist(-.004, -.384, .125, 0., .9)
        self.assertEqual(v, 0.)
        self.assertAlmostEqual(w, -.384)
        v, w = compensate_turn(v, w, turn_floor)
        self.assertEqual(v, 0.)
        self.assertAlmostEqual(w * 283.11258278, -200.)
        self.assertEqual(compensate_turn(0., 0., turn_floor), (0., 0.))
        self.assertEqual(compensate_turn(0., .001, turn_floor), (0., .001))
        self.assertEqual(compensate_turn(.1, .2, turn_floor), (.1, .2))
        self.assertEqual(compensate_turn(0., .8, turn_floor), (0., .8))

    def test_rotation_is_progress_but_commanded_rotation_without_motion_is_stalled(self):
        p = yaml.safe_load((PACKAGE / 'config/patrol_params.yaml').read_text())
        for yaw in (.044, .081):
            self.assertFalse(is_stalled((0.,0.,0.), (0.,0.,yaw), 3.1,
                                       p['stall_time'], p['stall_distance'], p['stall_yaw']))
        self.assertFalse(is_stalled((0.,0.,3.1), (0.,0.,-3.0), 3.1, 3., .02, p['stall_yaw']))
        self.assertTrue(is_stalled((0.,0.,0.), (.001,0.,.01), 3.1, 3., .02, p['stall_yaw']))
        self.assertFalse(is_stalled((0.,0.,0.), (0.,0.,0.), 3.1,
                                   p['near_goal_stall_time'], .02, p['stall_yaw']))
        self.assertTrue(is_stalled((0.,0.,0.), (0.,0.,0.), 6.1,
                                  p['near_goal_stall_time'], .02, p['stall_yaw']))


if __name__ == '__main__':
    unittest.main()
