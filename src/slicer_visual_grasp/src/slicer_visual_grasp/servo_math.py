# Copied unchanged from tested Vision_adjust/vision_core.py servo_command.
# Source SHA256: e103facebd4bde7376322a60a90295f78a9895d1ed66d98563b92cd2aa3f9ee1
import numpy as np

def servo_command(feature, target, jacobian, gain, max_linear, max_angular,
                  tolerance_px, max_reverse=0.0):
    """Return vx (m/s) and wz (rad/s) for calibrated image displacement.

    ``jacobian`` is 2x2: columns are image pixels/metre of forward motion and
    pixels/radian of yaw. ``gain`` is a nonnegative scalar in 1/s. The Euclidean
    image error is compared to ``tolerance_px``. A common scale factor enforces
    the supplied bounds. Reverse is enabled only by a positive effective Guard
    limit; otherwise solve again on the permitted forward-zero boundary.

    Status is active, reached, reverse_forbidden, invalid_input, or
    invalid_calibration. Every status except active returns zero velocity.
    """
    def stopped(status):
        return {"vx": 0.0, "wz": 0.0, "status": status}

    try:
        feature = np.asarray(feature, dtype=np.float64)
        target = np.asarray(target, dtype=np.float64)
        parameters = np.asarray((gain, max_linear, max_angular, tolerance_px, max_reverse),
                                dtype=np.float64)
    except (TypeError, ValueError, OverflowError):
        return stopped("invalid_input")
    if (feature.shape != (2,) or target.shape != (2,)
            or parameters.shape != (5,) or not np.all(np.isfinite(feature))
            or not np.all(np.isfinite(target))
            or not np.all(np.isfinite(parameters))):
        return stopped("invalid_input")
    gain, max_linear, max_angular, tolerance_px, max_reverse = parameters
    if (gain < 0 or max_linear <= 0 or max_angular <= 0
            or tolerance_px < 0 or max_reverse < 0):
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
    if velocity[0] < 0 and max_reverse == 0:
        try:
            angular = np.linalg.lstsq(jacobian[:, 1:2], requested_pixels,
                                      rcond=None)[0][0]
        except np.linalg.LinAlgError:
            return stopped("invalid_calibration")
        if not np.isfinite(angular):
            return stopped("invalid_calibration")
        if angular == 0:
            return stopped("reverse_forbidden")
        velocity = np.array((0.0, angular))
    scale = 1.0
    linear_limit = max_reverse if velocity[0] < 0 else max_linear
    if abs(velocity[0]) > linear_limit:
        scale = min(scale, linear_limit / abs(velocity[0]))
    if abs(velocity[1]) > max_angular:
        scale = min(scale, max_angular / abs(velocity[1]))
    return {"vx": float(velocity[0] * scale),
            "wz": float(velocity[1] * scale), "status": "active"}
