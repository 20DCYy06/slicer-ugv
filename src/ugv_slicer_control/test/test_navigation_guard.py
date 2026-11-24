#!/usr/bin/env python3
"""Pure Guard-policy regression: forbidden arcs stop instead of becoming spins."""
import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from navigation_core import (adaptive_forward_twist, body_twist_between, guarded_forward_twist,
                             planner_limits, saturate_twist, slew_arc, swept_clear)


class NavigationGuardTests(unittest.TestCase):
    SOURCE = dict(max_forward=.3, max_reverse=.02, max_angular=.6,
                  accel_linear=.5, accel_angular=2.)
    DWA = dict(max_vel_x=.125, min_vel_x=0., max_vel_trans=.125,
               max_vel_theta=.25, acc_lim_x=.2, acc_lim_trans=.2,
               acc_lim_theta=1.)

    def test_forward_speed_uses_checked_prefix_and_keeps_arc_curvature(self):
        pose = (0., 0., 0.)
        footprint = [(-.2, -.11), (.2, -.11), (.2, .11), (-.2, .11)]
        grid = dict(width=80, height=80, resolution=.05,
                    origin=(-2., -2., 0.), data=[0] * 6400)
        limits = dict(accel_linear=.5, accel_angular=1.)
        for wall_cost in (100, -1):
            for iy in range(grid['height']):
                grid['data'][iy * grid['width'] + 52] = wall_cost
            target = (.8, .3)
            self.assertFalse(swept_clear(pose, *target, footprint, grid, 2.1))
            output = adaptive_forward_twist(pose, target, (0., 0.), (.1, 0., .0375),
                                            footprint, grid, .8, .5, .5, .1, limits)
            self.assertGreater(output[0], 0.)
            self.assertLess(output[0], target[0])
            self.assertTrue(swept_clear(pose, *output, footprint, grid, 2.1))
            steady = adaptive_forward_twist(pose, target, (.30, .1125), (.1, 0., .0375),
                                            footprint, grid, .8, .5, .5, .1, limits)
            self.assertAlmostEqual(steady[1] / steady[0], target[1] / target[0])
            self.assertFalse(swept_clear(pose, *steady, footprint, grid, 2.1))
            self.assertTrue(swept_clear(pose, *steady, footprint, grid, max(.8, steady[0]/.5+.5)))
            self.assertIsNone(adaptive_forward_twist(
                pose, target, (0., 0.), (.8, 0., 0.), footprint, grid, .8, .5, .5, .1, limits))
            self.assertIsNone(adaptive_forward_twist(
                pose, target, (.8, 0.), (0., 0., 0.), footprint, grid, .8, .5, .5, .1, limits))
            self.assertIsNone(adaptive_forward_twist(
                pose, target, (0., 0.), None, footprint, grid, .8, .5, .5, .1, limits))

    def test_observation_boundary_slows_before_guard_has_to_stop(self):
        target = (.8, 0.)
        footprint = [(-.16, -.14), (.16, -.14), (.16, .14), (-.16, .14)]
        grid = dict(width=140, height=40, resolution=.05, origin=(-2., -1., 0.),
                    data=[0] * 5600)
        # Fixed observation limit 4.85 m ahead of the starting pose.
        for iy in range(grid['height']):
            grid['data'][iy * grid['width'] + 137] = -1
        limits = dict(accel_linear=.5, accel_angular=1.)
        dt = .05
        previous = (0., 0.)
        motor = [0.] * 4  # Last command takes 0.20 s to appear in odometry.
        position = 0.
        peak = 0.
        for _ in range(120):
            pose = (position, 0., 0.)
            measured = motor.pop(0)
            demand = slew_arc(previous, target, dt, .5, 1.)
            command = guarded_forward_twist(pose, target, demand, previous,
                                           (measured, 0., 0.), footprint, grid,
                                           .8, .2, .5, dt, limits)
            self.assertIsNotNone(command, f'avoidable stop at {position:.3f} m')
            peak = max(peak, command[0])
            self.assertTrue(swept_clear(pose, *command, footprint, grid,
                                        max(.8, command[0] / .2 + .5)))
            previous = command
            motor.append(command[0])
            position += measured * dt
        self.assertGreater(position, 2.5)
        self.assertAlmostEqual(peak, .8)

    def test_observed_side_motion_and_occupied_start_disallow_reduced_command(self):
        pose = (0., 0., 0.)
        footprint = [(-.2, -.11), (.2, -.11), (.2, .11), (-.2, .11)]
        grid = dict(width=80, height=80, resolution=.05,
                    origin=(-2., -2., 0.), data=[0] * 6400)
        limits = dict(accel_linear=.5, accel_angular=1.)
        for ix in range(grid['width']):
            grid['data'][46 * grid['width'] + ix] = 100
        self.assertFalse(swept_clear(pose, .1, 0., footprint, grid, 2.1, lateral=.3))
        self.assertIsNone(adaptive_forward_twist(
            pose, (.8, 0.), (0., 0.), (.1, .3, 0.), footprint, grid, .8, .5, .5, .1, limits))
        grid['data'][40 * grid['width'] + 40] = -1
        self.assertIsNone(adaptive_forward_twist(
            pose, (.8, 0.), (0., 0.), (0., 0., 0.), footprint, grid, .8, .5, .5, .1, limits))

    def test_observed_twist_uses_body_axes_and_wraps_yaw(self):
        vx, vy, angular = body_twist_between((0., 0., 0.), (.04, -.02, 0.), .1)
        self.assertAlmostEqual(vx, .4)
        self.assertAlmostEqual(vy, -.2)
        self.assertEqual(angular, 0.)
        vx, vy, angular = body_twist_between((0., 0., math.pi-.01),
                                             (-.05, 0., -math.pi+.01), .1)
        self.assertAlmostEqual(vx, .5000083334)
        self.assertAlmostEqual(vy, 0.)
        self.assertAlmostEqual(angular, .2)

    def test_dwa_uses_its_caps_and_forbids_reverse_without_teb_parameters(self):
        limits = planner_limits(self.SOURCE, 'dwa_local_planner/DWAPlannerROS', self.DWA)
        self.assertEqual(limits, dict(max_forward=.125, max_reverse=0., max_angular=.25,
                                     accel_linear=.2, accel_angular=1.))
        self.assertEqual(saturate_twist(-.001, .25, limits['max_forward'],
                                       limits['max_reverse'], limits['max_angular']), (0., 0.))

    def test_dwa_also_respects_source_and_translational_caps(self):
        source = dict(self.SOURCE, max_forward=.1, accel_angular=.4)
        planner = dict(self.DWA, max_vel_trans=.08, acc_lim_trans=.15)
        self.assertEqual(planner_limits(source, 'dwa_local_planner/DWAPlannerROS', planner),
                         dict(max_forward=.08, max_reverse=0., max_angular=.25,
                              accel_linear=.15, accel_angular=.4))
        self.assertEqual(source['max_forward'], .1)

    def test_external_reverse_opt_in_retains_source_and_translation_caps(self):
        for source_reverse, expected in ((.02, .02), (.3, .125), (0., 0.)):
            with self.subTest(source_reverse=source_reverse):
                limits = planner_limits(dict(self.SOURCE, max_reverse=source_reverse),
                                        'dwa_local_planner/DWAPlannerROS', self.DWA,
                                        external_reverse=True)
                self.assertEqual(limits['max_reverse'], expected)
                self.assertEqual(limits['max_forward'], .125)
                self.assertEqual(limits['max_angular'], .25)
                self.assertEqual(limits['accel_linear'], .2)
                self.assertEqual(limits['accel_angular'], 1.)
        limits = planner_limits(self.SOURCE, 'dwa_local_planner/DWAPlannerROS',
                                dict(self.DWA, max_vel_trans=.01), external_reverse=True)
        self.assertEqual(limits['max_reverse'], .01)

    def test_external_reverse_keeps_dwa_forward_only_and_requires_boolean(self):
        with self.assertRaisesRegex(ValueError, 'min_vel_x'):
            planner_limits(self.SOURCE, 'dwa_local_planner/DWAPlannerROS',
                           dict(self.DWA, min_vel_x=-.02), external_reverse=True)
        for invalid in (1, 'true', None):
            with self.subTest(external_reverse=invalid), self.assertRaises(ValueError):
                planner_limits(self.SOURCE, 'dwa_local_planner/DWAPlannerROS',
                               self.DWA, external_reverse=invalid)

    def test_reverse_sweep_checks_rear_occupied_unknown_and_outside_grid(self):
        footprint = [(-.2, -.15), (.2, -.15), (.2, .15), (-.2, .15)]
        grid = dict(width=100, height=100, resolution=.05,
                    origin=(-2.5, -2.5, 0.), data=[0] * 10000)
        self.assertTrue(swept_clear((0, 0, 0), -.5, 0., footprint, grid, 1.))
        for value in (100, -1):
            with self.subTest(rear_cell=value):
                grid['data'][50 * 100 + 39] = value  # x=[-.55,-.50], behind body.
                self.assertFalse(swept_clear((0, 0, 0), -.5, 0., footprint, grid, 1.))
        grid['data'][50 * 100 + 39] = 0
        self.assertFalse(swept_clear((-2., 0, 0), -.5, 0., footprint, grid, 1.))
        self.assertTrue(swept_clear((0, 0, 0), -.4, .4, footprint, grid, 1.))
        grid['data'][49 * 100 + 40] = 100  # rear cell reached by the curved sweep.
        self.assertFalse(swept_clear((0, 0, 0), -.4, .4, footprint, grid, 1.))

    def test_teb_reference_retains_source_and_selected_planner_bounds(self):
        teb = dict(max_vel_x=.2, max_vel_x_backwards=.01, max_vel_theta=.25,
                   acc_lim_x=.2, acc_lim_theta=1.)
        source = dict(self.SOURCE, max_forward=.1, max_reverse=0.)
        self.assertEqual(planner_limits(source, 'teb_local_planner/TebLocalPlannerROS', teb),
                         dict(max_forward=.1, max_reverse=0., max_angular=.25,
                              accel_linear=.2, accel_angular=1.))

    def test_unknown_plugin_is_rejected_instead_of_using_teb_limits(self):
        with self.assertRaisesRegex(ValueError, 'Unsupported local planner'):
            planner_limits(self.SOURCE, 'other/UnreviewedPlanner', self.DWA)

    def test_dwa_reverse_or_forward_floor_is_rejected(self):
        for minimum in (-.02, .01, math.nan, math.inf, False):
            with self.subTest(min_vel_x=minimum):
                with self.assertRaises(ValueError):
                    planner_limits(self.SOURCE, 'dwa_local_planner/DWAPlannerROS',
                                   dict(self.DWA, min_vel_x=minimum))

    def test_invalid_caps_are_rejected_before_any_output(self):
        for key in ('max_vel_x', 'max_vel_trans', 'max_vel_theta', 'acc_lim_x',
                    'acc_lim_trans', 'acc_lim_theta'):
            for invalid in (0., -.1, math.nan, math.inf, True):
                with self.subTest(key=key, value=invalid):
                    with self.assertRaises(ValueError):
                        planner_limits(self.SOURCE, 'dwa_local_planner/DWAPlannerROS',
                                       dict(self.DWA, **{key: invalid}))
        with self.assertRaises(ValueError):
            planner_limits(dict(self.SOURCE, max_reverse=-.02),
                           'dwa_local_planner/DWAPlannerROS', self.DWA)

    def test_forbidden_reverse_arc_rejects_the_whole_command(self):
        for velocity, omega in ((-.02, .25), (-.02, -.25), (-1.e-12, .05),
                                (-1.e-12, -.05), (-.02, 0.)):
            with self.subTest(vx=velocity, wz=omega):
                self.assertEqual(saturate_twist(velocity, omega, .125, 0., .25),
                                 (0., 0.))

    def test_original_pure_turn_remains_legal_in_both_directions(self):
        for velocity, omega in ((0., .25), (-0., -.25), (0., .02)):
            with self.subTest(vx=velocity, wz=omega):
                self.assertEqual(saturate_twist(velocity, omega, .125, 0., .25),
                                 (0., omega))

    def test_forward_saturation_preserves_arc_ratio(self):
        self.assertEqual(saturate_twist(.25, .25, .125, 0., .25), (.125, .125))
        self.assertEqual(saturate_twist(.1, -.5, .125, 0., .25), (.05, -.25))

    def test_positive_reverse_limit_preserves_existing_arc_ratio(self):
        self.assertEqual(saturate_twist(-.04, .2, .125, .02, .25), (-.02, .1))
        self.assertEqual(saturate_twist(-.01, -.5, .125, .02, .25), (-.005, -.25))

    def test_zero_and_rejected_command_stop_without_slew_residue(self):
        self.assertEqual(saturate_twist(0., 0., .125, 0., .25), (0., 0.))
        target = saturate_twist(-.02, .25, .125, 0., .25)
        self.assertEqual(slew_arc((.1, .25), target, .05, .2, 1.), (0., 0.))

    def test_non_finite_input_is_rejected(self):
        for velocity, omega in ((math.nan, .1), (.1, math.inf), (-math.inf, .1)):
            with self.subTest(vx=velocity, wz=omega):
                with self.assertRaises(ValueError):
                    saturate_twist(velocity, omega, .125, 0., .25)


if __name__ == '__main__':
    unittest.main()
