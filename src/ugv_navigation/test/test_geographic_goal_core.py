#!/usr/bin/env python3
"""Geographic client boundary checks without a ROS master or hardware."""
import importlib.util
import math
from pathlib import Path
import sys
import json
import tempfile
import unittest
from types import SimpleNamespace
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / 'scripts'), str(ROOT.parent / 'cup_gnss/src')]

try:
    import geographic_goal_core as core
    import geographic as cli
except ImportError:
    core = cli = None


class GeographicClientTests(unittest.TestCase):
    def setUp(self):
        self.assertIsNotNone(core, 'geographic goal helper must be implemented')

    def status(self, **updates):
        result = dict(state='ready', registered=True, generation=7, status_stamp=100.,
                      datum='WGS84', origin=dict(latitude=0., longitude=0., altitude_ellipsoid_m=0.),
                      latest_observation=dict(stamp=100., base_xy=[3., 4.], gga_quality=4,
                                              fresh_rtk_usable=True, covariance_assumed=True))
        result.update(updates)
        return result

    def test_enu_input_requires_an_asserted_origin_and_missing_yaw_is_unspecified(self):
        self.assertIsNotNone(cli)
        with self.assertRaises(SystemExit):
            cli.arguments(['send', '--east', '2', '--north', '3'])
        args = cli.arguments(['send', '--east', '2', '--north', '3', '--session-enu'])
        self.assertIsNone(args.yaw)
        target = core.resolve_target(args, self.status(), (2., 1., 0.))
        self.assertEqual(target['map_xyz'], [2., 3., 0.])
        self.assertAlmostEqual(target['yaw'], math.pi / 2.)
        self.assertEqual(target['physical_point'], 'base_link')
        explicit = cli.arguments(['send', '--east', '2', '--north', '3', '--session-enu', '--yaw', '-1.2'])
        self.assertAlmostEqual(core.resolve_target(explicit, self.status(), (2., 1., 0.))['yaw'], -1.2)

    def test_initialization_distance_is_finite_positive_without_an_arbitrary_site_limit(self):
        self.assertEqual(cli.arguments(['initialize', '--distance', '11']).distance, 11.)
        for distance in ('0', '-1', 'nan'):
            with self.subTest(distance=distance), self.assertRaises(SystemExit):
                cli.arguments(['initialize', '--distance', distance])

    def test_startup_roundtrip_is_limited_to_two_meters(self):
        args = cli.arguments(['initialize', '--return-to-start'])
        self.assertTrue(args.return_to_start)
        self.assertEqual(args.distance, 2.)
        self.assertEqual(cli.arguments(['initialize']).distance, 2.5)
        with self.assertRaises(SystemExit):
            cli.arguments(['initialize', '--distance', '2.1', '--return-to-start'])

    def test_roundtrip_return_command_is_bounded_and_rejects_departure_from_corridor(self):
        self.assertEqual(cli.registration_return_command((0., 0., 0.), (2., 0., 0.), 2.),
                         (-.8, 0.))
        self.assertEqual(cli.registration_return_command((0., 0., 0.), (.09, 0., 0.), 2.),
                         (0., 0.))
        approach, _ = cli.registration_return_command((0., 0., 0.), (.25, 0., 0.), 2.)
        self.assertGreater(approach, -.40)
        self.assertLess(approach, -.12)
        forward, turn = cli.registration_return_command((0., 0., 0.), (1., .1, 0.), 2.)
        self.assertLess(forward, 0.)
        self.assertGreater(turn, 0.)
        self.assertLessEqual(abs(turn), .15)
        for pose in ((-.2, 0., 0.), (1., .35, 0.), (1., 0., 1.), (3., 0., 0.)):
            with self.subTest(pose=pose), self.assertRaises(ValueError):
                cli.registration_return_command((0., 0., 0.), pose, 2.)

    def test_wgs84_target_at_origin_is_zero_without_antenna_offset(self):
        args = cli.arguments(['plan', '--latitude', '0', '--longitude', '0', '--altitude-ellipsoid', '0'])
        target = core.resolve_target(args, self.status(), (-3., -4., 0.))
        self.assertEqual(target['map_xyz'], [0., 0., 0.])
        self.assertAlmostEqual(target['yaw'], math.atan2(4., 3.))
        self.assertEqual(target['original']['latitude'], 0.)
        with self.assertRaises(SystemExit):
            cli.arguments(['send', '--latitude', '91', '--longitude', '0', '--altitude-ellipsoid', '0'])

    def test_foreign_enu_origin_and_explicit_heading_are_both_converted(self):
        # At latitude 30, moving source longitude 90 to task longitude 0
        # maps source East into task North. Known WGS84 geometry, not a
        # conversion helper computing its own expected answer.
        status = self.status(origin=dict(latitude=30., longitude=0., altitude_ellipsoid_m=0.))
        source = dict(datum='WGS84', axes='ENU', units='m',
                      origin=dict(latitude=30., longitude=90., altitude_ellipsoid_m=0.))
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'origin.json'
            path.write_text(json.dumps(source))
            args = cli.arguments(['plan', '--east', '0', '--north', '0', '--origin-file', str(path), '--yaw', '0'])
            target = core.resolve_target(args, status, (0., 0., 0.))
            self.assertAlmostEqual(target['map_xyz'][0], 5528256.639292835, places=4)
            self.assertAlmostEqual(target['map_xyz'][1], 2764128.3196464176, places=4)
            self.assertAlmostEqual(target['yaw'], math.pi/2., places=6)
            source['datum'] = 'CGCS2000'
            path.write_text(json.dumps(source))
            with self.assertRaisesRegex(ValueError, 'datum'):
                core.resolve_target(args, status, (0., 0., 0.))

    def test_status_receipt_header_generation_and_invalidation_cancel_only_owned_goal(self):
        status = self.status(state='degraded')
        self.assertIsNone(core.status_problem(status, 10., 10.2, 100.2, generation=7))
        for changed, received, now, ros, expected in [
            (self.status(registered=False), 10., 10.2, 100.2, 'registered'),
            (self.status(state='invalid'), 10., 10.2, 100.2, 'invalid'),
            (self.status(generation=8), 10., 10.2, 100.2, 'generation'),
            (status, 10., 12., 100.2, 'receive'),
            (status, 10., 10.2, 102., 'stamp')]:
            with self.subTest(expected=expected):
                self.assertIn(expected, core.status_problem(changed, received, now, ros, generation=7))

    def test_initialization_waits_for_a_fresh_qualified_stationary_gnss_sample(self):
        self.assertTrue(hasattr(core, 'registration_observation_problem'))
        self.assertIn('sample', core.registration_observation_problem({}, 100., 2.5))
        self.assertIn('stale', core.registration_observation_problem(
            {'registration_samples': 2, 'qualified_stamp': 90.}, 100., 2.5))
        self.assertIsNone(core.registration_observation_problem(
            {'registration_samples': 2, 'qualified_stamp': 99.}, 100., 2.5))

    def odom(self, end=10., speed=0.):
        return [dict(received=end-1.2+i*.1, stamp=100.+i*.1, speed=speed,
                     angular_speed=0., x=1., y=2.) for i in range(13)]

    def test_fresh_quiet_odom_tail_does_not_confirm_physical_stopping(self):
        self.assertTrue(core.quiet_odom_tail(self.odom(), 10.1, 101.3)['confirmed'])
        self.assertFalse(core.quiet_odom_tail(self.odom(speed=.1), 10.1, 101.3)['confirmed'])
        self.assertFalse(core.quiet_odom_tail(self.odom(), 11., 102.2)['confirmed'])
        self.assertFalse(core.quiet_odom_tail(self.odom()[-3:], 10.1, 101.3)['confirmed'])
        repeated = self.odom()
        for row in repeated:
            row['stamp'] = 101.2
        self.assertFalse(core.quiet_odom_tail(repeated, 10.1, 101.3)['confirmed'])

    def run_initialization_quiet(self, settles_after=None, invalidates_after=None):
        """Run the actual wait and quiet-tail check with a 20 Hz odometry clock."""
        clock = SimpleNamespace(ticks=0)
        client = object.__new__(cli.GeographicClient)
        client.stop_window, client.status_age = 1., 1.
        client.ros = SimpleNamespace(is_shutdown=lambda: False, Time=SimpleNamespace(
            now=lambda: SimpleNamespace(to_sec=lambda: 100.+clock.ticks*.05)))

        def observation():
            elapsed = clock.ticks*.05
            invalid = invalidates_after is not None and elapsed >= invalidates_after
            status = self.status(state='invalid' if invalid else 'collecting', registered=False,
                                 status_stamp=100.+elapsed, reason='TEST invalidation' if invalid else '')
            rows = [dict(received=10.+i*.05, stamp=100.+i*.05,
                         speed=0. if settles_after is not None and i*.05 >= settles_after else .03,
                         angular_speed=0., x=0., y=0.) for i in range(clock.ticks+1)]
            return status, 10.+elapsed, rows

        def sleep(_seconds):
            clock.ticks += 1

        client.observation = observation
        with patch.object(cli.time, 'monotonic', side_effect=lambda: 10.+clock.ticks*.05), \
                patch.object(cli.time, 'sleep', side_effect=sleep):
            result = client.quiet(10., generation=7, initializing=True)
        return result, clock.ticks*.05

    def test_initialization_quiet_allows_settling_before_a_full_two_second_window(self):
        # The field case settled about 2.05 s after Action success. A four-second
        # total wait ends with only about 1.9 s of quiet data and rejects it.
        result, elapsed = self.run_initialization_quiet(settles_after=2.05)
        self.assertTrue(result['confirmed'], result)
        self.assertGreaterEqual(result['observed_quiet_s'], 2.-1e-6)
        self.assertGreaterEqual(result['source_quiet_s'], 2.-1e-6)
        self.assertGreaterEqual(elapsed, 4.05-1e-6)
        self.assertLess(elapsed, 4.3)

    def test_initialization_quiet_continuous_motion_fails_with_a_bounded_wait(self):
        result, elapsed = self.run_initialization_quiet()
        self.assertFalse(result['confirmed'])
        self.assertEqual(result['observed_quiet_s'], 0.)
        self.assertGreaterEqual(elapsed, 4.)
        self.assertLessEqual(elapsed, 6.1)

    def test_initialization_quiet_state_invalidation_fails_without_waiting_for_deadline(self):
        result, elapsed = self.run_initialization_quiet(invalidates_after=.5)
        self.assertFalse(result['confirmed'])
        self.assertIn('initialization invalidated: TEST invalidation', result['reason'])
        self.assertLessEqual(elapsed, .55)

    def test_arrival_uses_qualified_original_gnss_and_keeps_action_and_stop_separate(self):
        target = dict(map_xyz=[0., 0., 0.], original={'latitude': 0., 'longitude': 0.})
        review = core.arrival_review(target, self.status(), 10., 10.1, 100.1,
                                     7, 3, {'confirmed': True, 'window_start_stamp': 99., 'window_end_stamp': 100.1}, tolerance=5.1)
        self.assertEqual(review['geographic']['horizontal_residual_m'], 5.)
        self.assertTrue(review['geographic']['confirmed'])
        self.assertTrue(review['geographic']['covariance_assumed'])
        review = core.arrival_review(target, self.status(latest_observation=dict(
            stamp=100., base_xy=[0., 0.], gga_quality=5, fresh_rtk_usable=False)),
            10., 10.1, 100.1, 7, 3, {'confirmed': True}, tolerance=.35)
        self.assertEqual(review['move_base']['state'], 3)
        self.assertTrue(review['stop']['confirmed'])
        self.assertFalse(review['geographic']['confirmed'])
        self.assertIsNone(review['geographic']['horizontal_residual_m'])
        degraded = core.arrival_review(target, self.status(state='degraded', reason='RTK Float'),
                                      10., 10.1, 100.1, 7, 3, {'confirmed': True}, tolerance=5.1)
        self.assertFalse(degraded['geographic']['confirmed'], 'a previous Fixed sample cannot override current degradation')

    def test_arrival_rejects_still_fresh_fix_before_stop_and_uses_new_actual_residual(self):
        rows = [dict(received=10.+i*.1, stamp=101.4+i*.1, speed=0.,
                     angular_speed=0., x=.396, y=0.) for i in range(11)]
        stopped = core.quiet_odom_tail(rows, 11., 102.4)
        self.assertTrue(stopped['confirmed'])
        target = dict(map_xyz=[0., 0., 0.])
        status = self.status(status_stamp=102.4, latest_observation=dict(
            stamp=100., base_xy=[.2, 0.], gga_quality=4, fresh_rtk_usable=True))
        old = core.arrival_review(target, status, 11., 11., 102.4, 7, 3, stopped)
        self.assertFalse(old['geographic']['confirmed'])
        self.assertIn('stopped window', old['geographic']['reason'])
        self.assertEqual(old['move_base']['state'], 3)
        self.assertTrue(old['stop']['confirmed'])
        self.assertIsNone(old['geographic']['horizontal_residual_m'])
        status['latest_observation'].update(stamp=102., base_xy=[.396, 0.])
        new = core.arrival_review(target, status, 11., 11., 102.4, 7, 3, stopped)
        self.assertTrue(new['geographic']['qualified'])
        self.assertFalse(new['geographic']['confirmed'])
        self.assertAlmostEqual(new['geographic']['horizontal_residual_m'], .396)
        status['latest_observation']['base_xy'] = [.2, 0.]
        self.assertTrue(core.arrival_review(target, status, 11., 11., 102.4, 7, 3, stopped)['geographic']['confirmed'])

    def test_stop_requires_explicit_window_and_never_accepts_a_later_outside_fix(self):
        target = dict(map_xyz=[3., 4., 0.])
        missing = core.arrival_review(target, self.status(), 10., 10., 100., 7, 3, {'confirmed': True})
        self.assertFalse(missing['geographic']['confirmed'])
        self.assertIn('window', missing['geographic']['reason'])
        window = dict(confirmed=True, window_start_stamp=98., window_end_stamp=99.9)
        later = core.arrival_review(target, self.status(), 10., 10., 100., 7, 3, window)
        self.assertFalse(later['geographic']['confirmed'])
        self.assertIn('stopped window', later['geographic']['reason'])

    def test_arrival_bounded_wait_uses_new_window_fix_or_reports_unconfirmed(self):
        for new_fix in (True, False):
            clock = SimpleNamespace(elapsed=0.)
            client = object.__new__(cli.GeographicClient)
            client.fix_age, client.status_age, client.stop_window, client.tolerance = 2.5, 1., 1., .35
            client.ros = SimpleNamespace(is_shutdown=lambda: False, Time=SimpleNamespace(
                now=lambda: SimpleNamespace(to_sec=lambda: 102.4+clock.elapsed)))

            def observation():
                elapsed = clock.elapsed
                rows = [dict(received=10.+i*.1, stamp=101.4+i*.1, speed=0.,
                             angular_speed=0., x=.396, y=0.) for i in range(11+int(elapsed/.1))]
                # Keep the latest row fresh even between odom ticks.
                obs = dict(stamp=100., base_xy=[.2, 0.], gga_quality=4, fresh_rtk_usable=True)
                if new_fix and elapsed >= .1:
                    obs.update(stamp=102.4, base_xy=[.396, 0.])
                return self.status(status_stamp=102.4+elapsed, latest_observation=obs), 11.+elapsed, rows

            client.observation = observation
            stopped = core.quiet_odom_tail(observation()[2], 11., 102.4)
            def sleep(seconds):
                clock.elapsed += seconds
            with patch.object(cli.time, 'monotonic', side_effect=lambda: 11.+clock.elapsed), patch.object(cli.time, 'sleep', side_effect=sleep):
                result = client.arrival_after_stop(dict(map_xyz=[0., 0., 0.], generation=7),
                                                   {'state': 3}, 9.)
            self.assertFalse(result['geographic']['confirmed'])
            self.assertTrue(result['stop']['confirmed'])
            if new_fix:
                self.assertLess(clock.elapsed, .2)
                self.assertAlmostEqual(result['geographic']['horizontal_residual_m'], .396)
            else:
                self.assertGreaterEqual(clock.elapsed, 2.5-1e-9)
                self.assertLess(clock.elapsed, 2.6)
                self.assertIn('unavailable', result['geographic']['reason'])

    def test_arrival_rechecks_current_motion_before_using_previous_quiet_window(self):
        client = object.__new__(cli.GeographicClient)
        client.fix_age, client.status_age, client.stop_window, client.tolerance = 2.5, 1., 1., .35
        client.ros = SimpleNamespace(is_shutdown=lambda: False, Time=SimpleNamespace(
            now=lambda: SimpleNamespace(to_sec=lambda: 100.1)))
        rows = [dict(received=9.+i*.1, stamp=99.+i*.1, speed=0.,
                     angular_speed=0., x=3., y=4.) for i in range(11)]
        previous = core.quiet_odom_tail(rows, 10., 100.)
        self.assertTrue(previous['confirmed'])
        rows.append(dict(received=10.1, stamp=100.1, speed=.2, angular_speed=0., x=3.02, y=4.))
        client.observation = lambda: (self.status(status_stamp=100.1), 10.1, rows)
        with patch.object(cli.time, 'monotonic', return_value=10.1):
            result = client.arrival_after_stop(dict(map_xyz=[3., 4., 0.], generation=7),
                                               {'state': 3}, 8.)
        self.assertFalse(result['stop']['confirmed'])
        self.assertFalse(result['geographic']['confirmed'])

    def capture_sample(self, stamp=100., yaw=math.pi/2.):
        fix = dict(stamp=stamp, stamp_ns=round(stamp*1e9), frame='gnss_link', status=2,
                   latitude=0., longitude=0., altitude=10.)
        quality = dict(stamp_ns=fix['stamp_ns'], gga_quality='4', fresh_rtk_usable='true',
                       fix_valid='true', session='receiver-1')
        poses = [dict(stamp=stamp-.1, x=8., y=9., yaw=yaw),
                 dict(stamp=stamp+.1, x=8., y=9., yaw=yaw),
                 dict(stamp=stamp+.2, x=8., y=9., yaw=0.)]
        origin = dict(latitude=0., longitude=0., altitude_ellipsoid_m=0.)
        return core.capture_sample(fix, quality, poses, [.5, .2], origin, stamp+.3, 2.5)

    def test_capture_pairs_raw_epoch_pose_and_compensates_rotated_lever_once(self):
        sample = self.capture_sample()
        self.assertAlmostEqual(sample['base_enu'][0], .2, places=7)
        self.assertAlmostEqual(sample['base_enu'][1], -.5, places=7)
        self.assertAlmostEqual(sample['base_enu'][2], 10., places=7)
        self.assertEqual(sample['stamp'], 100.)
        # Interpolation must use the fix epoch, not the latest yaw=0 above.
        self.assertAlmostEqual(sample['yaw'], math.pi/2.)
        fix = dict(stamp=100., stamp_ns=100000000000, frame='gnss_link', status=2,
                   latitude=0., longitude=0., altitude=10.)
        quality = dict(stamp_ns=1, gga_quality='4', fresh_rtk_usable='true', fix_valid='true')
        with self.assertRaisesRegex(ValueError, 'epoch'):
            core.capture_sample(fix, quality, [], [.5, .2], self.status()['origin'], 100.2, 2.5)
        quality['stamp_ns'] = fix['stamp_ns']
        with self.assertRaisesRegex(ValueError, 'odom'):
            core.capture_sample(fix, quality, [], [.5, .2], self.status()['origin'], 100.2, 2.5)
        quality['gga_quality'] = '5'
        with self.assertRaisesRegex(ValueError, 'fixed'):
            core.capture_sample(fix, quality, [], [.5, .2], self.status()['origin'], 100.2, 2.5)

    def test_capture_file_round_trip_is_portable_wgs84_and_height_is_not_base_height(self):
        samples = [self.capture_sample(stamp) for stamp in (100., 100.5, 101.)]
        stop = dict(confirmed=True, window_start_stamp=100., window_end_stamp=101.)
        record = core.capture_target(samples, self.status()['origin'], stop, 3.)
        self.assertEqual(record['physical_reference'], 'base_link_horizontal')
        self.assertIn('antenna', record['height_semantics'])
        self.assertEqual(record['capture']['sample_count'], 3)
        self.assertEqual(record['capture']['quality']['gga_quality'], 4)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'mark.json'
            path.write_text(json.dumps(record))
            for command in ('plan', 'send'):
                args = cli.arguments([command, '--target-file', str(path)])
                target = core.resolve_target(args, self.status(generation=99), (2., 1., 0.))
                self.assertAlmostEqual(target['map_xyz'][0], .2, places=7)
                self.assertAlmostEqual(target['map_xyz'][1], -.5, places=7)
                self.assertAlmostEqual(target['map_xyz'][2], 10., places=7)
                self.assertEqual(target['physical_point'], 'base_link')
            # A new initialization/origin must not bind the WGS84 file to the old session.
            new_status = self.status(generation=100, origin=dict(latitude=0., longitude=.00001, altitude_ellipsoid_m=0.))
            target = core.resolve_target(args, new_status, (2., 1., 0.))
            self.assertAlmostEqual(target['map_xyz'][0], -0.913196653, places=6)
            for invalid in ([str(path), '--east', '1', '--north', '2', '--session-enu'],
                            [str(path), '--origin-file', str(path)]):
                with self.assertRaises(SystemExit):
                    cli.arguments(['plan', '--target-file'] + invalid)

    def test_capture_does_not_initialize_send_action_or_publish_goal(self):
        self.assertEqual(cli.arguments(['capture', '--output', 'mark.json']).duration, 3.)
        with self.assertRaises(SystemExit):
            cli.arguments(['capture'])
        client = object.__new__(cli.GeographicClient)
        client.args = cli.arguments(['capture', '--duration', '.1', '--output', 'mark.json'])
        client.stop_window, client.fix_age, client.status_age = 1., 2.5, 1.
        client.fresh_status = lambda: self.status(antenna_in_base_link_xy_m=[.5, .2])
        client.trigger = lambda *a: self.fail('capture must not initialize')
        client.run_action = lambda *a, **k: self.fail('capture must not send Action')
        client.publish = lambda *a: self.fail('capture must not publish a goal')
        client.capture_inputs = lambda: ([], {}, [])
        # Full live success is checked by the isolated ROS probe; no-data must be bounded.
        times = iter([10., 20.])
        client.ros = SimpleNamespace(is_shutdown=lambda: False,
                                     Time=SimpleNamespace(now=lambda: SimpleNamespace(to_sec=lambda: 100.)))
        with patch.object(cli.time, 'monotonic', side_effect=lambda: next(times)):
            with self.assertRaisesRegex(ValueError, 'capture'):
                client.capture()

    def test_action_transport_exception_still_cancels_the_owned_goal(self):
        # Only the external Action transport is replaced; execute the actual
        # client method to catch losing ownership before exception cleanup.
        class Transport:
            cancelled = False

            def wait_for_server(self, duration):
                return True

            def send_goal(self, goal):
                pass

            def wait_for_result(self, duration):
                if not self.cancelled:
                    raise ValueError('transport interrupted')
                return True

            def cancel_goal(self):
                self.cancelled = True

        client = object.__new__(cli.GeographicClient)
        client.client = Transport()
        client.args = SimpleNamespace(timeout=5., action='/ugv/move_base')
        client.ros = SimpleNamespace(Duration=lambda seconds: seconds,
                                     Time=SimpleNamespace(now=lambda: SimpleNamespace(to_sec=lambda: 100.)))
        client.observation = lambda: (self.status(), 10., [])
        client.status_age = 1.
        client.MoveBaseGoal = lambda **kwargs: kwargs
        client.owned = False
        # Use the same receive clock as the actual method, avoiding a frozen
        # test ROS clock being mistaken for a stale subscription.
        import time
        client.observation = lambda: (self.status(), time.monotonic(), [])
        with self.assertRaisesRegex(ValueError, 'transport interrupted'):
            client.run_action(object(), 7)
        self.assertTrue(client.client.cancelled, 'an Action transport error must cancel this client goal')


if __name__ == '__main__':
    unittest.main()
