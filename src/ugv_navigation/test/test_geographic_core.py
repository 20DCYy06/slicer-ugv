#!/usr/bin/env python3
"""Deterministic geometry only: no ROS master, transport, devices or accuracy claim."""
import importlib
import json
import math
from pathlib import Path
import sys
import unittest


PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE / 'scripts'))
sys.path.insert(0, str(PACKAGE.parent / 'cup_gnss' / 'src'))
try:
    core = importlib.import_module('geographic_core')
except ModuleNotFoundError as error:
    if error.name != 'geographic_core':
        raise
    core = None


def odom(stamp, x, y, yaw=0.):
    return dict(stamp=stamp, x=x, y=y, yaw=yaw)


def pair(stamp, x, y, east, north, yaw=0.):
    return dict(stamp=stamp, odom=odom(stamp, x, y, yaw),
                enu=[east, north, 0.])


ORIGIN = dict(latitude=0., longitude=0., altitude_ellipsoid_m=0.)


class GeographicCoreTest(unittest.TestCase):
    def setUp(self):
        self.assertIsNotNone(core, 'The pure geographic_core module is required')

    def assertXY(self, actual, expected, places=8):
        self.assertEqual(len(actual), len(expected))
        for value, wanted in zip(actual, expected):
            self.assertAlmostEqual(value, wanted, places=places)

    def test_wrap_preserves_shortest_angle_across_both_pi_boundaries(self):
        for angle, expected in ((3 * math.pi, -math.pi),
                                (-3 * math.pi, -math.pi),
                                (2 * math.pi + .2, .2),
                                (-2 * math.pi - .2, -.2)):
            with self.subTest(angle=angle):
                self.assertAlmostEqual(core.wrap(angle), expected)

    def test_transform_rotates_before_translation_for_cardinal_axes(self):
        for yaw, expected in ((0., [12., 21.]), (math.pi / 2., [9., 22.]),
                              (math.pi, [8., 19.]), (-math.pi / 2., [11., 18.])):
            with self.subTest(yaw=yaw):
                self.assertXY(core.transform_xy(2., 1., yaw, 10., 20.), expected)

    def test_antenna_and_base_reference_apply_rotated_lever_once(self):
        antenna = core.antenna_odom(odom(1., 1., 2., math.pi / 2.), [.5, .2])
        self.assertXY(antenna, [.8, 2.5])
        self.assertXY(core.base_from_antenna([8., 9., 40.], math.pi / 2., [.5, .2]),
                      [8.2, 8.5])

    def test_enu_to_wgs84_inverse_preserves_horizontal_point_and_explicit_up(self):
        from cup_gnss.enu import to_enu
        for origin in (ORIGIN, dict(latitude=30., longitude=120., altitude_ellipsoid_m=50.)):
            for point in ((0., 0., 0.), (3., -4., 1.5), (-12., 9., -2.)):
                lat, lon, height = core.enu_to_geodetic(point, origin)
                actual = to_enu(lat, lon, height, origin)
                for a, b in zip(actual, point):
                    self.assertAlmostEqual(a, b, places=7)
        lat, lon, height = core.enu_to_geodetic((0., 0., 10.), ORIGIN)
        self.assertEqual(lat, 0.)
        self.assertEqual(lon, 0.)
        self.assertEqual(height, 10.)

    def test_interpolation_wraps_yaw_and_keeps_requested_stamp(self):
        samples = [odom(2., 1., -2., math.radians(170.)),
                   odom(2.4, 3., 4., math.radians(-170.))]
        result = core.interpolate_odom(samples, 2.2)
        self.assertAlmostEqual(result['stamp'], 2.2)
        self.assertXY([result['x'], result['y']], [2., 1.])
        self.assertAlmostEqual(result['yaw'], -math.pi)
        self.assertEqual(core.interpolate_odom(samples, 2.)['stamp'], 2.)
        self.assertEqual(core.interpolate_odom([samples[0]], 2.)['x'], 1.)

    def test_interpolation_rejects_extrapolation_gaps_and_disordered_time(self):
        cases = [([], 1.), ([odom(1., 0., 0.)], 1.01),
                 ([odom(1., 0., 0.), odom(1.4, 1., 0.)], .9),
                 ([odom(1., 0., 0.), odom(1.6, 1., 0.)], 1.3),
                 ([odom(2., 0., 0.), odom(1., 1., 0.)], 1.5),
                 ([odom(1., 0., 0.), odom(1., 1., 0.)], 1.)]
        for samples, stamp in cases:
            with self.subTest(samples=samples, stamp=stamp):
                with self.assertRaises(ValueError):
                    core.interpolate_odom(samples, stamp)

    def test_alignment_handles_north_west_south_east_with_nonzero_odom_yaw(self):
        # Odom base starts at (3,4), points north, with lever (.2,.1).
        # Its antenna is (2.9,4.2), then (2.9,5.2), then (2.9,6.2).
        fixtures = [
            (0., [(12.9, 2.2), (12.9, 3.2), (12.9, 4.2)]),
            (math.pi / 2., [(5.8, .9), (4.8, .9), (3.8, .9)]),
            (-math.pi, [(7.1, -6.2), (7.1, -7.2), (7.1, -8.2)]),
            (-math.pi / 2., [(14.2, -4.9), (15.2, -4.9), (16.2, -4.9)]),
        ]
        for yaw, points in fixtures:
            with self.subTest(yaw=yaw):
                samples = [pair(float(i), 3., 4. + i, east, north, math.pi / 2.)
                           for i, (east, north) in enumerate(points)]
                result = core.fit_alignment(samples, [.2, .1])
                self.assertAlmostEqual(core.wrap(result['yaw'] - yaw), 0.)
                self.assertXY(result['translation'], [10., -2.])
                self.assertAlmostEqual(result['rmse_m'], 0.)
                self.assertAlmostEqual(result['displacement_m'], 2.)
                self.assertEqual(result['span_s'], 2.)
                self.assertEqual(result['samples'], 3)
                self.assertGreater(result['heading_std_rad'], 0.)
                json.dumps(result, allow_nan=False)

    def test_alignment_handles_positive_and_negative_non_cardinal_rotation(self):
        root_two = math.sqrt(2.)
        for yaw, points in ((math.pi / 4., [(4., -3.), (4., -3. + root_two),
                                           (4., -3. + 2. * root_two)]),
                            (-math.pi / 4., [(4., -3.), (4. + root_two, -3.),
                                            (4. + 2. * root_two, -3.)])):
            samples = [pair(float(i), float(i), float(i), *point, yaw=.3)
                       for i, point in enumerate(points)]
            result = core.fit_alignment(samples, [0., 0.])
            self.assertAlmostEqual(result['yaw'], yaw)
            self.assertXY(result['translation'], [4., -3.])

    def test_alignment_compensates_each_turning_antenna_in_odom_frame(self):
        samples = [pair(0., 0., 0., 9.8, 20.4, 0.),
                   pair(1., 2., 0., 9.6, 21.8, math.pi / 2.),
                   pair(2., 2., 2., 8.2, 21.6, math.pi)]
        result = core.fit_alignment(samples, [.4, .2])
        self.assertAlmostEqual(result['yaw'], math.pi / 2.)
        self.assertXY(result['translation'], [10., 20.])
        self.assertAlmostEqual(result['rmse_m'], 0.)

    def test_alignment_rejects_stationary_short_motion_and_turn_in_place(self):
        cases = [([pair(0., 0., 0., 4., 5.), pair(1., 0., 0., 4.1, 5.1)], [0., 0.]),
                 ([pair(0., 0., 0., 4., 5.), pair(1., .2, 0., 4.2, 5.)], [0., 0.]),
                 ([pair(0., 0., 0., 1., 0.),
                   pair(1., 0., 0., -1., 0., math.pi)], [1., 0.])]
        for samples, lever in cases:
            with self.subTest(samples=samples):
                with self.assertRaises(ValueError):
                    core.fit_alignment(samples, lever)

    def test_alignment_reports_residual_and_rejects_inconsistent_trajectories(self):
        samples = [pair(0., -1., 0., 4., -1.7), pair(1., 0., 0., 5., -2.6),
                   pair(2., 1., 0., 6., -1.7)]
        result = core.fit_alignment(samples, [0., 0.])
        self.assertAlmostEqual(result['yaw'], 0.)
        self.assertXY(result['translation'], [5., -2.])
        self.assertAlmostEqual(result['rmse_m'], math.sqrt(.18))
        self.assertGreater(result['heading_std_rad'], math.radians(5.))
        with self.assertRaises(ValueError):
            core.fit_alignment(samples, [0., 0.], max_residual=.1)

    def test_alignment_rejects_unpaired_or_duplicate_times(self):
        samples = [pair(0., 0., 0., 0., 0.), pair(1., 1., 0., 1., 0.)]
        samples[1]['odom']['stamp'] = 1.2
        with self.assertRaises(ValueError):
            core.fit_alignment(samples, [0., 0.])
        samples[1]['stamp'] = 0.
        samples[1]['odom']['stamp'] = 0.
        with self.assertRaises(ValueError):
            core.fit_alignment(samples, [0., 0.])

    def curved_track(self, delay=0.):
        def flight(t):
            x = .35*t
            y = .9*math.sin(.45*t)**2
            return x, y, math.atan2(.405*math.sin(.9*t), .35)

        local = [odom(i*.10, *flight(i*.10)) for i in range(101)]
        pairs = []
        for i in range(50):
            stamp = .07+i*.20
            x, y, yaw = flight(stamp-delay)
            # Independently construct a rotated antenna and a 90-degree map transform.
            antenna_x = x+.37*math.cos(yaw)+.12*math.sin(yaw)
            antenna_y = y+.37*math.sin(yaw)-.12*math.cos(yaw)
            pairs.append(dict(stamp=stamp, odom=core.interpolate_odom(local, stamp),
                              enu=[4.-antenna_y, -2.+antenna_x, 0.]))
        return pairs

    def test_auto_alignment_uses_async_curve_and_rejects_isolated_outlier(self):
        pairs = self.curved_track()
        pairs[0]['enu'][0] += .12
        pairs[-1]['enu'][0] -= .12
        for sample in pairs[15:18]:
            sample['enu'][0] += 1.5
        fitted, progress = core.fit_trajectory_alignment(pairs, [.37, -.12])
        self.assertTrue(progress['ready'], progress)
        self.assertGreater(fitted['samples'], 6)
        self.assertGreater(fitted['rejected_samples'], 0)
        self.assertAlmostEqual(fitted['yaw'], math.pi/2., delta=.035)
        self.assertAlmostEqual(fitted['translation'][0], 4., delta=.07)
        self.assertAlmostEqual(fitted['translation'][1], -2., delta=.07)

    def test_auto_alignment_accepts_a_straight_trajectory(self):
        samples = [pair(i*.5, .20*i, 0., 10., 20.+.20*i, 0.) for i in range(16)]
        fitted, progress = core.fit_trajectory_alignment(samples, [0., 0.])
        self.assertTrue(progress['ready'], progress)
        self.assertAlmostEqual(fitted['yaw'], math.pi/2.)

    def test_auto_alignment_rejects_dense_near_or_turning_in_place_samples(self):
        samples = [pair(i*.02, .04*math.sin(i*.2), 0., math.cos(i*.1), math.sin(i*.1), i*.1)
                   for i in range(150)]
        fitted, progress = core.fit_trajectory_alignment(samples, [1., 0.])
        self.assertIsNone(fitted)
        self.assertFalse(progress['ready'])
        self.assertLess(progress['samples'], 3)

    def test_auto_alignment_does_not_bridge_missing_paired_data(self):
        samples = [pair(t, x, 0., x, 0.) for t, x in
                   ((0., 0.), (.4, .25), (.8, .5), (4., 2.), (4.4, 2.25), (4.8, 2.5))]
        fitted, progress = core.fit_trajectory_alignment(samples, [0., 0.])
        self.assertIsNone(fitted)
        self.assertLess(progress['baseline_m'], 1.)

    def test_auto_alignment_rejects_delayed_curve_without_clock_correction(self):
        fitted, progress = core.fit_trajectory_alignment(self.curved_track(delay=.9), [.37, -.12])
        self.assertIsNone(fitted, progress)
        self.assertFalse(progress['ready'])

    def test_auto_alignment_reuses_evidence_across_verified_stationary_resume(self):
        samples = [pair(i*.5, .20*i, 0., .20*i, 0.) for i in range(6)]
        anchor = pair(30., 1.02, 0., 1.02, 0.)
        anchor['resumed'] = True
        samples.append(anchor)
        samples.extend(pair(30.+i*.5, 1.+.20*i, 0., 1.+.20*i, 0.) for i in range(1, 5))
        fitted, progress = core.fit_trajectory_alignment(samples, [0., 0.])
        self.assertTrue(progress['ready'], progress)
        self.assertAlmostEqual(fitted['yaw'], 0.)
        anchor['enu'][0] += .5
        fitted, progress = core.fit_trajectory_alignment(samples, [0., 0.])
        self.assertIsNone(fitted, progress)

    def test_enu_conversion_preserves_identical_origin_and_height_difference(self):
        self.assertXY(core.enu_between([10., 20., 30.], ORIGIN, ORIGIN), [10., 20., 30.])
        high = dict(ORIGIN, altitude_ellipsoid_m=100.)
        self.assertXY(core.enu_between([10., 20., 30.], ORIGIN, high), [10., 20., -70.])

    def test_enu_conversion_rotates_distinct_origins_through_ecef(self):
        # At (0,0), ENU=(2,3,4) gives ECEF=(a+4,2,3). At (0,90),
        # east=-ECEF.x, north=ECEF.z, up=ECEF.y-a.
        target = dict(ORIGIN, longitude=90.)
        self.assertXY(core.enu_between([2., 3., 4.], ORIGIN, target),
                      [-6378141., 3., -6378135.], places=6)
        polar = dict(ORIGIN, latitude=90.)
        self.assertXY(core.enu_between([2., 3., 4.], polar, ORIGIN),
                      [2., 6356756.314245179, -6378140.], places=6)

    def test_invalid_numeric_ranges_and_shapes_fail_without_output(self):
        invalid_origins = [dict(ORIGIN, latitude=91.), dict(ORIGIN, longitude=-181.),
                           dict(ORIGIN, altitude_ellipsoid_m=float('nan')), {}]
        for origin in invalid_origins:
            with self.subTest(origin=origin):
                with self.assertRaises(ValueError):
                    core.enu_between([0., 0., 0.], origin, ORIGIN)
        operations = [lambda: core.wrap(float('inf')),
                      lambda: core.transform_xy(True, 0., 0., 0., 0.),
                      lambda: core.antenna_odom(odom(1., 0., 0.), [1.]),
                      lambda: core.base_from_antenna([0., 0., float('nan')], 0., [0., 0.]),
                      lambda: core.interpolate_odom([odom(1., 0., 0.)], 1., max_gap=0.),
                      lambda: core.fit_alignment([], [0., 0.]),
                      lambda: core.fit_alignment([pair(0., 0., 0., 0., 0.),
                                                  pair(1., 1., 0., 1., 0.)],
                                                 [0., 0.], min_displacement=0.)]
        for operation in operations:
            with self.assertRaises(ValueError):
                operation()


if __name__ == '__main__':
    unittest.main(verbosity=2)
