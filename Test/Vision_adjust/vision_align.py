#!/usr/bin/env python3
"""Click a point/edge and observe it; ROS control is explicit and calibrated."""
import argparse
import csv
from pathlib import Path
import sys
import time

import cv2
import numpy as np
import yaml

from vision_core import PointTracker, servo_command

RAW_TOPIC = "/cmd_vel/nav_raw"
GUARD_NODE = "/slicer_nav_arc_guard"
DWA_NAMESPACE = "/ugv/move_base/DWAPlannerROS"


def require_control_config(source, config):
    if source != "ros":
        raise ValueError("--control requires a live ROS Image source")
    result = servo_command((0, 0), (0, 0), config.get("jacobian"), 1, 1, 1, 0)
    if result["status"] == "invalid_calibration":
        raise ValueError("--control requires a measured, finite, nonsingular 2x2 jacobian")


def control_bounds(get_param):
    limits = tuple(float(get_param(DWA_NAMESPACE + "/" + key))
                   for key in ("max_vel_x", "max_vel_theta"))
    if not all(np.isfinite(value) and value > 0 for value in limits):
        raise ValueError("actual DWA max_vel_x/max_vel_theta must be finite and positive")
    try:
        reverse = float(get_param(GUARD_NODE + "/effective_max_reverse"))
    except KeyError:
        reverse = 0.0  # Older/default Guard retains the original forward-only policy.
    if not np.isfinite(reverse) or reverse < 0:
        raise ValueError("effective Guard reverse limit must be finite and nonnegative")
    return limits + (reverse,)


def check_control_route(system_state_reply, caller_id, resolved_guard_input):
    if resolved_guard_input != RAW_TOPIC:
        raise ValueError("Slicer Guard input_topic must resolve to " + RAW_TOPIC)
    code, message, state = system_state_reply
    if code != 1:
        raise ValueError("cannot inspect ROS publishers: " + message)
    publishers = state[0]
    others = {node for topic, nodes in publishers if topic == RAW_TOPIC
              for node in nodes if node != caller_id}
    if others:
        raise ValueError("other raw velocity publishers: " + ", ".join(sorted(others)))


def frame_is_fresh(stamp, previous_stamp, ros_now, received_at, now, timeout):
    # Camera hardware-clock mapping can lead host ROS time by a few ms.
    # Keep the original header; bound clock skew and real receipt age separately.
    values = (stamp, ros_now, received_at, now, timeout)
    return bool(all(np.isfinite(value) for value in values)
                and stamp > 0 and timeout > 0
                and (previous_stamp is None or stamp > previous_stamp)
                and abs(ros_now - stamp) <= timeout
                and 0 <= now - received_at <= timeout)


def image_target(target, shape):
    height, width = shape[:2]
    if target is None:
        return width // 2, height // 2
    target = np.asarray(target, dtype=float)
    if (target.shape != (2,) or not np.all(np.isfinite(target))
            or not 0 <= target[0] < width or not 0 <= target[1] < height):
        raise ValueError("target must be two pixels inside the image")
    return int(target[0]), int(target[1])


def update_view(view, tracker, packet, valid):
    """Keep both edge clicks on one displayed frame until selection completes."""
    if view["clicks"]:
        return False
    frame, stamp, received = packet
    if not valid:
        tracker.reset()
    if frame is not None:
        view["frame"] = frame
    view["stamp"] = stamp
    view["received"] = received
    view["result"] = tracker.update(frame)
    return True


def select_feature(view, tracker, point, required_points):
    if view["frame"] is None:
        return
    if not view["clicks"]:
        tracker.reset()
        view["result"] = tracker.update(None)
    view["clicks"].append(point)
    if len(view["clicks"]) == required_points:
        view["result"] = tracker.select(view["frame"], view["clicks"])
        view["clicks"].clear()


def frame_command(view, target, config, bounds, fresh, paused, packet_received):
    feature = view["result"]["feature"]
    status = ("paused" if paused else "selecting_edge" if view["clicks"] else
              "stale" if not fresh or view["received"] != packet_received else
              "no_feature" if feature is None else None)
    if status:
        return {"vx": 0.0, "wz": 0.0, "status": status}
    return servo_command(feature, target, config["jacobian"], config["gain"],
                         bounds[0], bounds[1], config["tolerance_px"],
                         max_reverse=bounds[2])


def log_values(view, target, command, hostnow):
    feature = view["result"]["feature"] if view["result"]["valid"] else ("", "")
    return (view["stamp"], hostnow, *feature, *target,
            command["vx"], command["wz"], command["status"])


def window_closed(window):
    """GTK3 lacks WND_PROP_VISIBLE; our AUTOSIZE window has a supported flag."""
    try:
        return cv2.getWindowProperty(window, cv2.WND_PROP_AUTOSIZE) < 0
    except cv2.error:
        return True


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group()
    source.add_argument("--image", metavar="FILE", help="offline still image")
    source.add_argument("--video", metavar="FILE", help="offline video file")
    source.add_argument("--topic", metavar="ROS_IMAGE", help="ROS Image (default: config image_topic)")
    parser.add_argument("--config", default=str(Path(__file__).with_name("config.yaml")))
    parser.add_argument("--target", nargs=2, type=int, metavar=("X", "Y"))
    parser.add_argument("--feature", choices=("point", "edge"), default="point")
    parser.add_argument("--control", action="store_true", help="publish calibrated commands to fixed Guard raw input")
    parser.add_argument("--log", metavar="CSV", help="append image/host times, feature, target and command to CSV")
    args = parser.parse_args(argv)
    capture = subscriber = publisher = rospy = None
    log_stream = None
    zero = {"vx": 0.0, "wz": 0.0, "status": "stopped"}

    def publish(command):
        if publisher is not None:
            message = Twist()
            message.linear.x, message.angular.z = command["vx"], command["wz"]
            publisher.publish(message)

    try:
        with open(args.config, encoding="utf-8") as stream:
            config = yaml.safe_load(stream)
        if not isinstance(config, dict):
            raise ValueError("config must be a YAML mapping")
        for key in ("gain", "tolerance_px", "frame_timeout_s"):
            value = float(config[key])
            if not np.isfinite(value) or value < 0 or (key == "frame_timeout_s" and value == 0):
                raise ValueError(key + " must be finite and nonnegative (timeout positive)")
            config[key] = value
        kind = "image" if args.image else "video" if args.video else "ros"
        if args.control:
            require_control_config(kind, config)
        if args.log:
            log_stream = open(args.log, "a", newline="", encoding="utf-8")
            log_writer = csv.writer(log_stream)
            if log_stream.tell() == 0:
                log_writer.writerow(("image_stamp", "host_processing_time", "feature_x", "feature_y",
                                     "target_x", "target_y", "vx", "wz", "status"))
        packet = [None]
        bounds = None

        def check_ros_control():
            from rosgraph.names import resolve_name
            guard_input = resolve_name(rospy.get_param(GUARD_NODE + "/input_topic"), GUARD_NODE)
            check_control_route(rospy.get_master().getSystemState(), rospy.get_name(), guard_input)
            return control_bounds(rospy.get_param)

        if kind == "ros":
            import rospy
            from cv_bridge import CvBridge, CvBridgeError
            from sensor_msgs.msg import Image
            rospy.init_node("vision_align", anonymous=True)
            bridge = CvBridge()

            def receive(message):
                received = time.monotonic()
                try:
                    frame = bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
                except CvBridgeError:
                    frame = None
                packet[0] = (frame, message.header.stamp.to_sec(), received)

            subscriber = rospy.Subscriber(args.topic or config["image_topic"], Image,
                                          receive, queue_size=1, buff_size=2 ** 24)
            if args.control:
                from geometry_msgs.msg import Twist
                bounds = check_ros_control()  # No publisher is created before this passes.
                publisher = rospy.Publisher(RAW_TOPIC, Twist, queue_size=1)
                rospy.on_shutdown(lambda: publish(zero))
        elif kind == "image":
            frame = cv2.imread(args.image, cv2.IMREAD_COLOR)
            if frame is None:
                raise ValueError("cannot read image: " + args.image)
            packet[0] = (frame, 0.0, time.monotonic())
        else:
            capture = cv2.VideoCapture(args.video)
            if not capture.isOpened():
                raise ValueError("cannot read video: " + args.video)

        window = "Vision align: OBSERVE" if not args.control else "Vision align: CONTROL"
        cv2.namedWindow(window, cv2.WINDOW_AUTOSIZE)
        tracker = PointTracker()
        view = {"frame": None, "stamp": None, "received": None, "clicks": [], "result": tracker.update(None)}
        required_points = 1 if args.feature == "point" else 2

        def click(event, x, y, flags, userdata):
            if event != cv2.EVENT_LBUTTONDOWN or view["frame"] is None:
                return
            select_feature(view, tracker, (x, y), required_points)

        cv2.setMouseCallback(window, click)
        paused, frame_ok = False, False
        last_received = previous_stamp = dimensions = None
        last_route_check = time.monotonic()
        command = zero
        target = (0, 0)
        while rospy is None or not rospy.is_shutdown():
            if kind == "video" and not paused and not view["clicks"]:
                ok, frame = capture.read()
                if not ok:
                    break
                packet[0] = (frame, 0.0, time.monotonic())
            latest = packet[0]
            if latest is not None and latest[2] != last_received and not view["clicks"]:
                frame, stamp, received = latest
                last_received = received
                frame_ok = frame is not None
                if kind == "ros":
                    frame_ok = frame_ok and frame_is_fresh(
                        stamp, previous_stamp, rospy.get_time(), received,
                        time.monotonic(), config["frame_timeout_s"])
                    previous_stamp = max(previous_stamp or 0.0, stamp)
                update_view(view, tracker, latest, frame_ok)
                if frame is not None:
                    if dimensions != frame.shape[:2]:
                        first_frame = dimensions is None
                        dimensions = frame.shape[:2]
                        target = image_target(args.target if args.target is not None else config.get("target"), dimensions)
                        tracker.reset()
                        view["clicks"].clear()
                        for name, value, maximum in (("target X", target[0], dimensions[1] - 1),
                                                      ("target Y", target[1], dimensions[0] - 1)):
                            if not first_frame:
                                cv2.setTrackbarMax(name, window, maximum)
                                cv2.setTrackbarPos(name, window, value)
                            else:
                                cv2.createTrackbar(name, window, value, maximum, lambda value: None)
            fresh = frame_ok
            if kind == "ros" and latest is not None:
                fresh = fresh and frame_is_fresh(latest[1], None, rospy.get_time(),
                                                 latest[2], time.monotonic(), config["frame_timeout_s"])
            if view["frame"] is None:
                canvas = np.zeros((240, 640, 3), dtype=np.uint8)
            else:
                canvas = view["frame"].copy()
                target = (cv2.getTrackbarPos("target X", window), cv2.getTrackbarPos("target Y", window))
                cv2.drawMarker(canvas, target, (0, 255, 255), cv2.MARKER_CROSS, 18, 2)
            result = view["result"]
            for point in view["clicks"]:
                cv2.circle(canvas, point, 3, (0, 200, 255), 1)
            if result["valid"]:
                points = [tuple(int(round(v)) for v in point) for point in result["points"]]
                for point in points:
                    cv2.circle(canvas, point, 5, (0, 255, 0), 2)
                if len(points) == 2:
                    cv2.line(canvas, points[0], points[1], (0, 255, 0), 2)
                feature = tuple(int(round(v)) for v in result["feature"])
                cv2.line(canvas, feature, target, (255, 180, 0), 1)
                error = np.linalg.norm(np.asarray(target) - result["feature"])
                detail = "feature=(%.1f,%.1f) error=%.1fpx" % (*result["feature"], error)
            else:
                detail = result["reason"] + "; click " + str(required_points) + " point(s)"
            for row, text in enumerate(("target=%s  %s" % (target, detail),
                                         "status=%s  vx=%.3f wz=%.3f" % (command["status"], command["vx"], command["wz"]),
                                         "space: pause/resume  r: reselect  Esc: quit")):
                cv2.putText(canvas, text, (10, 22 + row * 23), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            cv2.imshow(window, canvas)
            key = cv2.waitKey(30) & 0xff
            if key == 27 or window_closed(window):
                break
            if key == ord(" "):
                paused = not paused
            elif key == ord("r"):
                tracker.reset()
                view["clicks"].clear()
                view["result"] = tracker.update(None)
            if args.control:
                if time.monotonic() - last_route_check >= 1.0:
                    bounds = check_ros_control()
                    last_route_check = time.monotonic()
                fresh = frame_ok and latest is not None and frame_is_fresh(
                    latest[1], None, rospy.get_time(), latest[2], time.monotonic(), config["frame_timeout_s"])
                target = (cv2.getTrackbarPos("target X", window), cv2.getTrackbarPos("target Y", window)) if dimensions else target
                command = frame_command(view, target, config, bounds, fresh, paused,
                                        latest[2] if latest is not None else None)
                publish(command)
            else:
                command = {**zero, "status": "paused" if paused else "selecting_edge" if view["clicks"] else "observing" if fresh else "stale"}
            if log_stream is not None:
                log_writer.writerow(log_values(view, target, command, time.time()))
                log_stream.flush()
        return 0
    except KeyboardInterrupt:
        return 0
    except Exception as error:
        print("vision_align: " + str(error), file=sys.stderr)
        return 1
    finally:
        try:
            publish(zero)
        except Exception as error:
            print("vision_align: final zero publish failed: " + str(error), file=sys.stderr)
        finally:
            if log_stream is not None:
                log_stream.close()
            if subscriber is not None:
                subscriber.unregister()
            if capture is not None:
                capture.release()
            cv2.destroyAllWindows()


if __name__ == "__main__":
    sys.exit(main())
