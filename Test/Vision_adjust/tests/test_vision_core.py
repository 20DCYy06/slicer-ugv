"""Pure, deterministic checks for local tracking and velocity calculation."""

import importlib
import importlib.util
from pathlib import Path
import sys
import unittest

import cv2
import numpy as np


CORE_DIRECTORY = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(CORE_DIRECTORY))


class CoreTest(unittest.TestCase):
    def setUp(self):
        self.assertIsNotNone(importlib.util.find_spec("vision_core"),
                             "vision_core must provide the local servo API")
        self.core = importlib.import_module("vision_core")


class PointTrackerTests(CoreTest):
    def setUp(self):
        super().setUp()
        self.tracker = self.core.PointTracker()
        rng = np.random.RandomState(32)
        self.frame = cv2.GaussianBlur(
            rng.randint(0, 256, (140, 180), dtype=np.uint8), (3, 3), 0)

    @staticmethod
    def translated(frame, dx, dy):
        return cv2.warpAffine(frame, np.float32([[1, 0, dx], [0, 1, dy]]),
                              (frame.shape[1], frame.shape[0]))

    def test_tracks_selected_point_on_translated_texture(self):
        selected = self.tracker.select(self.frame, [(70, 65)])
        self.assertTrue(selected["valid"])
        self.assertEqual(selected["feature"], (70.0, 65.0))
        result = self.tracker.update(self.translated(self.frame, 6, -4))
        self.assertTrue(result["valid"], result["reason"])
        np.testing.assert_allclose(result["feature"], (76, 61), atol=0.15)
        np.testing.assert_allclose(self.tracker.points, [(76, 61)], atol=0.15)

    def test_edge_feature_is_mean_of_tracked_endpoints(self):
        frame = np.zeros((140, 180), dtype=np.uint8)
        cv2.rectangle(frame, (50, 50), (110, 90), 255, -1)
        selected = self.tracker.select(frame, [(50, 50), (110, 50)])
        self.assertTrue(selected["valid"], selected["reason"])
        self.assertEqual(selected["feature"], (80.0, 50.0))
        result = self.tracker.update(self.translated(frame, 4, -2))
        self.assertTrue(result["valid"], result["reason"])
        np.testing.assert_allclose(result["feature"], (84, 48), atol=0.15)
        np.testing.assert_allclose(result["points"], [(54, 48), (114, 48)],
                                   atol=0.15)

    def test_accepts_bgr_camera_frame(self):
        bgr = cv2.cvtColor(self.frame, cv2.COLOR_GRAY2BGR)
        self.assertTrue(self.tracker.select(bgr, [(70, 65)])["valid"])
        result = self.tracker.update(self.translated(bgr, 2, 3))
        self.assertTrue(result["valid"], result["reason"])
        np.testing.assert_allclose(result["feature"], (72, 68), atol=0.15)

    def test_rejects_invalid_selection_and_clears_previous_selection(self):
        for points in ([], [(1, 2), (3, 4), (5, 6)], [(180, 20)],
                       [(-1, 50)], [(20, float("nan"))], [(20,)],
                       [(70, 65), (70, 65)]):
            with self.subTest(points=points):
                self.tracker.select(self.frame, [(70, 65)])
                result = self.tracker.select(self.frame, points)
                self.assertFalse(result["valid"])
                self.assertIsNone(result["feature"])
                self.assertEqual(result["points"], [])
                self.assertTrue(result["reason"])
                self.assertIsNone(self.tracker.points)

    def test_rejects_flat_selected_patch_without_moving_to_texture(self):
        frame = np.zeros((140, 180), dtype=np.uint8)
        frame[20:65, 20:65] = self.frame[20:65, 20:65]
        result = self.tracker.select(frame, [(125, 95)])
        self.assertFalse(result["valid"])
        self.assertIsNone(self.tracker.points)

    def test_lost_feature_requires_explicit_reselection(self):
        self.tracker.select(self.frame, [(70, 65)])
        result = self.tracker.update(np.zeros_like(self.frame))
        self.assertFalse(result["valid"])
        self.assertIsNone(self.tracker.points)
        self.assertFalse(self.tracker.update(self.frame)["valid"])

    def test_feature_leaving_image_invalidates_tracking(self):
        selected = self.tracker.select(self.frame, [(8, 65)])
        self.assertTrue(selected["valid"], selected["reason"])
        result = self.tracker.update(self.translated(self.frame, -20, 0))
        self.assertFalse(result["valid"])
        self.assertIsNone(result["feature"])
        self.assertIsNone(self.tracker.points)

    def test_unrelated_background_is_not_accepted_as_selected_feature(self):
        rng = np.random.RandomState(19)
        unrelated = cv2.GaussianBlur(
            rng.randint(0, 256, self.frame.shape, dtype=np.uint8), (3, 3), 0)
        self.tracker.select(self.frame, [(70, 65)])
        result = self.tracker.update(unrelated)
        self.assertFalse(result["valid"])
        self.assertIsNone(self.tracker.points)

    def test_target_update_changes_command_without_resetting_feature(self):
        selected = self.tracker.select(self.frame, [(70, 65)])
        first = self.core.servo_command(selected["feature"], (80, 65),
                                       ((100, 0), (0, 100)), 1, 1, 1, 0)
        second = self.core.servo_command(selected["feature"], (90, 75),
                                        ((100, 0), (0, 100)), 1, 1, 1, 0)
        self.assertAlmostEqual(first["vx"], 0.1)
        self.assertAlmostEqual(first["wz"], 0.0)
        self.assertAlmostEqual(second["vx"], 0.2)
        self.assertAlmostEqual(second["wz"], 0.1)
        self.assertEqual(self.tracker.points, [(70.0, 65.0)])
        updated = self.tracker.update(self.frame)
        self.assertTrue(updated["valid"], updated["reason"])
        np.testing.assert_allclose(updated["feature"], (70, 65), atol=0.15)

    def test_rejects_frame_size_change_and_invalid_frames(self):
        for frame in (self.frame[:80], None, np.empty((0, 0), np.uint8),
                      np.zeros((140, 180), np.float32)):
            with self.subTest(shape=getattr(frame, "shape", None)):
                self.tracker.select(self.frame, [(70, 65)])
                result = self.tracker.update(frame)
                self.assertFalse(result["valid"])
                self.assertIsNone(self.tracker.points)

    def test_reset_clears_selection(self):
        self.tracker.select(self.frame, [(70, 65)])
        self.tracker.reset()
        self.assertIsNone(self.tracker.points)
        result = self.tracker.update(self.frame)
        self.assertFalse(result["valid"])
        self.assertIsNone(result["feature"])


class ServoCommandTests(CoreTest):
    def command(self, feature=(0, 0), target=(10, 10),
                jacobian=((100, 0), (0, 100)), gain=1.0,
                max_linear=1.0, max_angular=1.0, tolerance_px=0.0,
                max_reverse=None):
        if max_reverse is not None:
            return self.core.servo_command(feature, target, jacobian, gain,
                                           max_linear, max_angular, tolerance_px,
                                           max_reverse=max_reverse)
        return self.core.servo_command(feature, target, jacobian, gain,
                                       max_linear, max_angular, tolerance_px)

    def assert_stopped(self, result, status):
        self.assertEqual(result["vx"], 0.0)
        self.assertEqual(result["wz"], 0.0)
        self.assertEqual(result["status"], status)

    def test_full_inverse_jacobian_preserves_coupling_and_gain(self):
        result = self.command(target=(15, -10),
                              jacobian=((100, 50), (0, -100)), gain=2.0)
        self.assertEqual(result["status"], "active")
        self.assertAlmostEqual(result["vx"], 0.2)
        self.assertAlmostEqual(result["wz"], 0.2)

    def test_axis_swap_and_angular_sign_follow_calibration(self):
        result = self.command(target=(20, 10),
                              jacobian=((0, 100), (100, 0)))
        self.assertAlmostEqual(result["vx"], 0.1)
        self.assertAlmostEqual(result["wz"], 0.2)
        result = self.command(jacobian=((100, 0), (0, -100)))
        self.assertAlmostEqual(result["vx"], 0.1)
        self.assertAlmostEqual(result["wz"], -0.1)

    def test_uniform_bounds_preserve_command_direction(self):
        result = self.command(target=(100, 200), max_linear=0.2,
                              max_angular=0.1)
        self.assertAlmostEqual(result["vx"], 0.05)
        self.assertAlmostEqual(result["wz"], 0.1)

    def test_configured_bounds_have_no_hidden_lower_cap(self):
        result = self.command(target=(100, 40), max_linear=0.8,
                              max_angular=0.5)
        self.assertAlmostEqual(result["vx"], 0.8)
        self.assertAlmostEqual(result["wz"], 0.32)

    def test_reverse_required_stops_both_commands(self):
        self.assert_stopped(self.command(target=(-10, 0)), "reverse_forbidden")

    def test_explicit_reverse_limit_uses_full_calibrated_inverse(self):
        result = self.command(target=(-10, 10), max_reverse=0.2)
        self.assertEqual(result["status"], "active")
        self.assertAlmostEqual(result["vx"], -0.1)
        self.assertAlmostEqual(result["wz"], 0.1)

    def test_reverse_limit_scales_both_axes_without_changing_arc(self):
        result = self.command(target=(-100, 200), max_reverse=0.2,
                              max_angular=0.5)
        self.assertAlmostEqual(result["vx"], -0.2)
        self.assertAlmostEqual(result["wz"], 0.4)

    def test_invalid_reverse_limit_stops(self):
        for limit in (-0.1, float("nan"), float("inf")):
            with self.subTest(limit=limit):
                self.assert_stopped(self.command(max_reverse=limit), "invalid_input")

    def test_reverse_with_angular_error_turns_without_claiming_reached(self):
        result = self.command(target=(-10, 10), tolerance_px=5)
        self.assertEqual(result["status"], "active")
        self.assertEqual(result["vx"], 0.0)
        self.assertAlmostEqual(result["wz"], 0.1)
        self.assert_stopped(self.command(feature=(0, 10), target=(-10, 10),
                                         tolerance_px=5), "reverse_forbidden")

    def test_recorded_near_goal_error_uses_legal_turn_to_reduce_pixel_error(self):
        jacobian = np.array(((310.2871199, 792.8549618),
                             (-3455.1373696, -507.6815472)))
        feature = np.array((198.78, 94.80))
        target = np.array((206.0, 91.0))
        error = target - feature
        unconstrained = np.linalg.solve(jacobian, 2 * error)
        self.assertLess(unconstrained[0], 0)
        result = self.command(feature=feature, target=target,
                              jacobian=jacobian, gain=2, max_linear=0.4,
                              max_angular=0.25, tolerance_px=5)
        self.assertEqual(result["status"], "active")
        self.assertEqual(result["vx"], 0.0)
        self.assertGreater(result["wz"], 0)
        velocity = np.array((result["vx"], result["wz"]))
        self.assertLess(np.linalg.norm(error - 0.05 * jacobian @ velocity),
                        np.linalg.norm(error))
        self.assertLess(np.linalg.norm(error - jacobian @ velocity / 2), 1)
        self.assertNotAlmostEqual(result["wz"], unconstrained[1])

    def test_reverse_boundary_is_constrained_least_squares_optimum(self):
        jacobian = np.array(((3.0, 2.0), (-4.0, 1.0)))
        requested = jacobian @ np.array((-0.2, 0.4))
        result = self.command(target=requested, jacobian=jacobian)
        velocity = np.array((result["vx"], result["wz"]))
        residual = jacobian @ velocity - requested
        self.assertEqual(result["status"], "active")
        self.assertEqual(result["vx"], 0.0)
        self.assertAlmostEqual(jacobian[:, 1] @ residual, 0.0)
        self.assertGreaterEqual(jacobian[:, 0] @ residual, 0.0)
        for forward in (0.0, 0.1, 0.5):
            for angular_offset in (-0.3, 0.0, 0.3):
                candidate = (forward, result["wz"] + angular_offset)
                self.assertLessEqual(np.linalg.norm(residual),
                                     np.linalg.norm(jacobian @ candidate
                                                    - requested) + 1e-12)

    def test_reverse_boundary_turn_respects_existing_angular_bound(self):
        result = self.command(target=(-100, 200), max_linear=0.2,
                              max_angular=0.1)
        self.assertEqual(result["status"], "active")
        self.assertEqual(result["vx"], 0.0)
        self.assertAlmostEqual(result["wz"], 0.1)

    def test_forward_trajectory_keeps_unconstrained_commands(self):
        jacobian = np.array(((100.0, 50.0), (0.0, -100.0)))
        target = np.array((15.0, -10.0))
        feature = np.zeros(2)
        for _ in range(5):
            expected = np.linalg.solve(jacobian, 2 * (target - feature))
            result = self.command(feature=feature, target=target,
                                  jacobian=jacobian, gain=2)
            self.assertEqual(result["status"], "active")
            np.testing.assert_allclose((result["vx"], result["wz"]), expected)
            feature += 0.05 * jacobian @ expected

    def test_euclidean_pixel_tolerance_and_zero_error_stop(self):
        self.assert_stopped(self.command(target=(3, 4), tolerance_px=5),
                            "reached")
        self.assert_stopped(self.command(target=(0, 0)), "reached")
        self.assertEqual(self.command(target=(3, 4), tolerance_px=4)["status"],
                         "active")

    def test_singular_or_invalid_calibration_stops_explicitly(self):
        for jacobian in (((1, 2), (2, 4)), ((1, 2, 3), (4, 5, 6)),
                         ((float("nan"), 0), (0, 1)), None):
            with self.subTest(jacobian=jacobian):
                self.assert_stopped(self.command(jacobian=jacobian),
                                    "invalid_calibration")

    def test_nonfinite_error_and_invalid_parameters_stop_explicitly(self):
        for kwargs in ({"feature": (float("nan"), 0)},
                       {"target": (0, float("inf"))},
                       {"feature": (1,)}, {"gain": float("nan")},
                       {"gain": -1}, {"max_linear": 0},
                       {"max_linear": -1}, {"max_angular": float("inf")},
                       {"max_angular": 0}, {"tolerance_px": -1}):
            with self.subTest(kwargs=kwargs):
                self.assert_stopped(self.command(**kwargs), "invalid_input")


if __name__ == "__main__":
    unittest.main()
