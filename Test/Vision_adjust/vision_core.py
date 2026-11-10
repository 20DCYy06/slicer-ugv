"""Local image tracking and calibrated planar visual-servo calculations.

This module performs no ROS communication and owns no control timers. Tracking
uses the selected pixels directly; a lost feature always needs reselection.
"""

import cv2
import numpy as np


class PointTracker:
    """Track one clicked point or both endpoints of a clicked edge.

    Frames must be uint8 grayscale, BGR or BGRA images. Selection and updates
    return a result dictionary; ``points`` is None whenever tracking is invalid.
    A two-point feature is the arithmetic mean of the tracked endpoints.
    """

    def __init__(self):
        self.reset()

    @property
    def points(self):
        if self._points is None:
            return None
        return [tuple(float(value) for value in point)
                for point in self._points.reshape(-1, 2)]

    def reset(self):
        self._points = None
        self._previous = None

    def _result(self, reason="ok"):
        points = self.points
        feature = None
        if points is not None:
            feature = tuple(float(value) for value in np.mean(points, axis=0))
        return {"valid": points is not None, "feature": feature,
                "points": points if points is not None else [], "reason": reason}

    def _invalidate(self, reason):
        self.reset()
        return self._result(reason)

    @staticmethod
    def _gray(frame):
        if not isinstance(frame, np.ndarray) or frame.dtype != np.uint8:
            return None
        if frame.ndim not in (2, 3) or not frame.size:
            return None
        if frame.ndim == 2:
            return np.ascontiguousarray(frame)
        if frame.shape[2] == 1:
            return np.ascontiguousarray(frame[:, :, 0])
        if frame.shape[2] == 3:
            return cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        if frame.shape[2] == 4:
            return cv2.cvtColor(frame, cv2.COLOR_BGRA2GRAY)
        return None

    @staticmethod
    def _inside(points, frame):
        points = points.reshape(-1, 2)
        return bool(np.all(np.isfinite(points))
                    and np.all(points[:, 0] >= 0)
                    and np.all(points[:, 0] < frame.shape[1])
                    and np.all(points[:, 1] >= 0)
                    and np.all(points[:, 1] < frame.shape[0]))

    @classmethod
    def _flow(cls, previous, current, points):
        options = {
            "winSize": (21, 21), "maxLevel": 3,
            "criteria": (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT,
                         30, 0.01),
            "minEigThreshold": 1e-4,
        }
        try:
            forward, status, error = cv2.calcOpticalFlowPyrLK(
                previous, current, points, None, **options)
            if (forward is None or status is None or error is None
                    or not np.all(status) or not cls._inside(forward, current)
                    or not np.all(np.isfinite(error)) or np.any(error > 20.0)):
                return None
            backward, status, error = cv2.calcOpticalFlowPyrLK(
                current, previous, forward, None, **options)
            if (backward is None or status is None or error is None
                    or not np.all(status) or not np.all(np.isfinite(backward))
                    or not np.all(np.isfinite(error)) or np.any(error > 20.0)):
                return None
            if np.any(np.linalg.norm((backward - points).reshape(-1, 2),
                                     axis=1) > 1.0):
                return None
        except cv2.error:
            return None
        return forward

    def select(self, frame, points):
        """Select exactly one point or two distinct endpoints without snapping."""
        self.reset()
        gray = self._gray(frame)
        if gray is None:
            return self._result("invalid_frame")
        try:
            selected = np.asarray(points, dtype=np.float32)
        except (TypeError, ValueError, OverflowError):
            return self._result("invalid_selection")
        if selected.shape == (2,):
            selected = selected.reshape(1, 2)
        if (selected.shape not in ((1, 2), (2, 2))
                or not self._inside(selected, gray)):
            return self._result("invalid_selection")
        if len(selected) == 2 and np.array_equal(selected[0], selected[1]):
            return self._result("invalid_selection")
        selected = selected.reshape(-1, 1, 2).copy()
        if self._flow(gray, gray, selected) is None:
            return self._result("feature_untrackable")
        self._points = selected
        self._previous = gray.copy()
        return self._result()

    def update(self, frame):
        """Advance the selected feature or invalidate it on any tracking loss."""
        if self._points is None:
            return self._result("no_selection")
        gray = self._gray(frame)
        if gray is None:
            return self._invalidate("invalid_frame")
        if gray.shape != self._previous.shape:
            return self._invalidate("frame_size_changed")
        tracked = self._flow(self._previous, gray, self._points)
        if tracked is None:
            return self._invalidate("feature_lost")
        self._points = tracked
        self._previous = gray.copy()
        return self._result()


def servo_command(feature, target, jacobian, gain, max_linear, max_angular,
                  tolerance_px):
    """Return vx (m/s) and wz (rad/s) for calibrated image displacement.

    ``jacobian`` is 2x2: columns are image pixels/metre of forward motion and
    pixels/radian of yaw. ``gain`` is a nonnegative scalar in 1/s. The Euclidean
    image error is compared to ``tolerance_px``. A common scale factor enforces
    the supplied positive bounds. Reverse requests stop both axes because the
    current Guard policy forbids reverse motion.

    Status is active, reached, reverse_forbidden, invalid_input, or
    invalid_calibration. Every status except active returns zero velocity.
    """
    def stopped(status):
        return {"vx": 0.0, "wz": 0.0, "status": status}

    try:
        feature = np.asarray(feature, dtype=np.float64)
        target = np.asarray(target, dtype=np.float64)
        parameters = np.asarray((gain, max_linear, max_angular, tolerance_px),
                                dtype=np.float64)
    except (TypeError, ValueError, OverflowError):
        return stopped("invalid_input")
    if (feature.shape != (2,) or target.shape != (2,)
            or parameters.shape != (4,) or not np.all(np.isfinite(feature))
            or not np.all(np.isfinite(target))
            or not np.all(np.isfinite(parameters))):
        return stopped("invalid_input")
    gain, max_linear, max_angular, tolerance_px = parameters
    if gain < 0 or max_linear <= 0 or max_angular <= 0 or tolerance_px < 0:
        return stopped("invalid_input")
    try:
        jacobian = np.asarray(jacobian, dtype=np.float64)
    except (TypeError, ValueError, OverflowError):
        return stopped("invalid_calibration")
    if jacobian.shape != (2, 2) or not np.all(np.isfinite(jacobian)):
        return stopped("invalid_calibration")
    with np.errstate(over="ignore", invalid="ignore"):
        error = target - feature
        requested_pixels = gain * error
    if not np.all(np.isfinite(error)) or not np.all(np.isfinite(requested_pixels)):
        return stopped("invalid_input")
    try:
        velocity = np.linalg.solve(jacobian, requested_pixels)
    except np.linalg.LinAlgError:
        return stopped("invalid_calibration")
    if not np.all(np.isfinite(velocity)):
        return stopped("invalid_calibration")
    if np.hypot(error[0], error[1]) <= tolerance_px:
        return stopped("reached")
    if velocity[0] < 0:
        return stopped("reverse_forbidden")
    scale = 1.0
    if abs(velocity[0]) > max_linear:
        scale = min(scale, max_linear / abs(velocity[0]))
    if abs(velocity[1]) > max_angular:
        scale = min(scale, max_angular / abs(velocity[1]))
    return {"vx": float(velocity[0] * scale),
            "wz": float(velocity[1] * scale), "status": "active"}
