"""Offline adapter checks; never import ROS or open a camera/window."""
import importlib.util
from pathlib import Path
import sys
import unittest

import cv2
import numpy as np

APP_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(APP_DIR))


def load_app():
    path = APP_DIR / "vision_align.py"
    if not path.exists():
        return None
    spec = importlib.util.spec_from_file_location("vision_align", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class VisionAdapterTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = load_app()

    def helper(self, name):
        self.assertIsNotNone(self.app, "vision_align adapter is not implemented")
        self.assertTrue(hasattr(self.app, name), name + " is not implemented")
        return getattr(self.app, name)

    def test_control_requires_ros_and_nonsingular_calibration(self):
        validate = self.helper("require_control_config")
        for source, jacobian in [("image", [[1, 0], [0, 1]]),
                                 ("ros", None), ("ros", [[1, 2], [2, 4]]),
                                 ("ros", [[1, 0], [0, float("nan")]])]:
            with self.subTest(source=source, jacobian=jacobian):
                with self.assertRaises(ValueError):
                    validate(source, {"jacobian": jacobian})
        validate("ros", {"jacobian": [[-120, 15], [40, 60]]})

    def test_bounds_use_actual_dwa_values_and_reject_invalid_limits(self):
        bounds = self.helper("control_bounds")
        parameters = {"/ugv/move_base/DWAPlannerROS/max_vel_x": 0.6,
                      "/ugv/move_base/DWAPlannerROS/max_vel_theta": 1.0}
        self.assertEqual(bounds(parameters.__getitem__), (0.6, 1.0))
        for bad in [0, -0.1, float("nan"), float("inf")]:
            parameters["/ugv/move_base/DWAPlannerROS/max_vel_x"] = bad
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                bounds(parameters.__getitem__)

    def test_route_rejects_other_raw_publishers_or_guard_mismatch(self):
        check = self.helper("check_control_route")
        check((1, "ok", [[["/cmd_vel/nav_raw", ["/vision"]]], [], []]),
              "/vision", "/cmd_vel/nav_raw")
        for publishers, guard in [
                ([["/cmd_vel/nav_raw", ["/vision", "/ugv/move_base"]]], "/cmd_vel/nav_raw"),
                ([], "/cmd_vel/nav"),
        ]:
            with self.subTest(publishers=publishers, guard=guard):
                with self.assertRaises(ValueError):
                    check((1, "ok", [publishers, [], []]), "/vision", guard)
        with self.assertRaises(ValueError):
            check((-1, "master unavailable", []), "/vision", "/cmd_vel/nav_raw")

    def test_live_frame_requires_advancing_stamp_and_both_age_limits(self):
        fresh = self.helper("frame_is_fresh")
        self.assertTrue(fresh(10.1, 10.0, 10.2, 50.0, 50.1, 0.25))
        for stamp, previous, ros_now, received, now in [
                (10.0, 10.0, 10.1, 50.0, 50.1),
                (9.9, 10.0, 10.1, 50.0, 50.1),
                (9.8, 9.7, 10.1, 50.0, 50.1),
                (10.1, 10.0, 10.2, 49.5, 50.1),
                (10.2, 10.0, 10.1, 50.0, 50.1),
                (0.0, None, 0.1, 50.0, 50.1),
        ]:
            with self.subTest(stamp=stamp, previous=previous, received=received):
                self.assertFalse(fresh(stamp, previous, ros_now, received, now, 0.25))

    def test_target_defaults_to_image_center_and_rejects_outside_image(self):
        target = self.helper("image_target")
        self.assertEqual(target(None, (480, 640)), (320, 240))
        self.assertEqual(target([30, 40], (480, 640)), (30, 40))
        for value in [[-1, 2], [640, 2], [1, float("nan")], [1]]:
            with self.subTest(value=value), self.assertRaises(ValueError):
                target(value, (480, 640))

    def test_pause_stale_or_lost_feature_always_stops_both_axes(self):
        command = self.helper("frame_command")
        config = {"jacobian": [[10, 0], [0, 10]], "gain": 0.5,
                  "tolerance_px": 5}
        for feature, fresh, paused, expected in [
                ((10, 10), True, True, "paused"),
                ((10, 10), False, False, "stale"),
                (None, True, False, "no_feature")]:
            with self.subTest(expected=expected):
                view = {"result": {"feature": feature}, "received": 50.0, "clicks": []}
                self.assertEqual(command(view, (30, 20), config, (0.6, 1.0),
                                         fresh, paused, 50.0),
                                 {"vx": 0.0, "wz": 0.0, "status": expected})
        view = {"result": {"feature": (10, 10)}, "received": 50.0, "clicks": []}
        self.assertEqual(command(view, (30, 20), config, (0.6, 1.0), True, False, 50.0),
                         {"vx": 0.6, "wz": 0.3, "status": "active"})

    def edge_frames(self):
        rng = np.random.RandomState(32)
        frame = np.zeros((140, 180), dtype=np.uint8)
        cv2.line(frame, (50, 65), (100, 65), 128, 2)
        for x in (50, 100):
            frame[50:80, x - 15:x + 15] = rng.randint(0, 256, (30, 30), dtype=np.uint8)
        frame = cv2.GaussianBlur(frame, (3, 3), 0)
        shifted = cv2.warpAffine(frame, np.float32([[1, 0, 20], [0, 1, 0]]),
                                 (180, 140))
        return frame, shifted

    def test_edge_clicks_share_frozen_frame_then_resume_tracking(self):
        update = self.helper("update_view")
        select = self.helper("select_feature")
        frame, shifted = self.edge_frames()
        tracker = self.app.PointTracker()
        view = {"frame": None, "received": None, "clicks": [],
                "result": tracker.update(None)}
        self.assertTrue(update(view, tracker, (frame, 10.0, 50.0), True))
        select(view, tracker, (50, 65), 2)
        self.assertFalse(update(view, tracker, (shifted, 10.1, 50.1), True))
        self.assertIs(view["frame"], frame)
        self.assertEqual(view["received"], 50.0)
        select(view, tracker, (100, 65), 2)
        self.assertTrue(view["result"]["valid"], view["result"]["reason"])
        self.assertEqual(view["result"]["points"], [(50.0, 65.0), (100.0, 65.0)])
        self.assertEqual(view["result"]["feature"], (75.0, 65.0))
        self.assertTrue(update(view, tracker, (shifted, 10.1, 50.1), True))
        self.assertTrue(view["result"]["valid"], view["result"]["reason"])
        np.testing.assert_allclose(view["result"]["feature"], (95, 65), atol=0.15)
        self.assertEqual(view["received"], 50.1)

    def test_edge_selection_stops_until_feature_matches_live_packet(self):
        update = self.helper("update_view")
        select = self.helper("select_feature")
        frame, shifted = self.edge_frames()
        tracker = self.app.PointTracker()
        view = {"frame": None, "received": None, "clicks": [],
                "result": tracker.update(None)}
        config = {"jacobian": [[100, 0], [0, 100]], "gain": 0.5,
                  "tolerance_px": 5}
        update(view, tracker, (frame, 10.0, 50.0), True)
        select(view, tracker, (50, 65), 2)
        pending = self.app.frame_command(view, (130, 65), config, (0.6, 1.0),
                                         True, False, 50.1)
        self.assertEqual(pending, {"vx": 0.0, "wz": 0.0, "status": "selecting_edge"})
        update(view, tracker, (shifted, 10.1, 50.1), True)
        select(view, tracker, (100, 65), 2)
        old_feature = self.app.frame_command(view, (130, 65), config, (0.6, 1.0),
                                             True, False, 50.1)
        self.assertEqual(old_feature, {"vx": 0.0, "wz": 0.0, "status": "stale"})
        update(view, tracker, (shifted, 10.1, 50.1), True)
        resumed = self.app.frame_command(view, (130, 65), config, (0.6, 1.0),
                                         True, False, 50.1)
        self.assertEqual(resumed["status"], "active")
        self.assertAlmostEqual(resumed["vx"], 0.175, delta=0.001)
        self.assertAlmostEqual(resumed["wz"], 0.0, delta=0.001)
        stale = self.app.frame_command(view, (130, 65), config, (0.6, 1.0),
                                       False, False, 50.1)
        self.assertEqual(stale, {"vx": 0.0, "wz": 0.0, "status": "stale"})


if __name__ == "__main__":
    unittest.main()
