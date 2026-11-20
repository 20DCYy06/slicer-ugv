#!/usr/bin/env python3
"""Publish Point 记录位置；2D Nav Goal 的专用录点话题记录 yaw，不占用 /initialpose。"""
import math
from pathlib import Path
import sys
import threading
import json
import yaml
sys.path.insert(0, str(Path(__file__).resolve().parent))
import rospy
import tf
from geometry_msgs.msg import PointStamped, PoseStamped
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion
from navigation_core import append_waypoint, finite


def main():
    rospy.init_node('slicer_waypoint_recorder')
    filename = rospy.get_param('~file')
    frame = rospy.get_param('~frame_id', 'map')
    yaw = finite(rospy.get_param('~clicked_yaw', 0.0), 'clicked_yaw')
    wait = finite(rospy.get_param('~wait_s', 2.0), 'wait_s')
    listener, lock = tf.TransformListener(), threading.Lock()
    status = rospy.Publisher('/slicer/waypoint_record/status', String, queue_size=100, latch=True)

    def record(message, has_yaw):
        rospy.loginfo('Received %s in frame=%s; saving waypoint...',
                      '2D Nav Goal' if has_yaw else 'map click', message.header.frame_id)
        try:
            if not message.header.frame_id:
                raise ValueError('Input frame_id is empty')
            with lock:
                if has_yaw:
                    value = message if message.header.frame_id == frame else listener.transformPose(frame, message)
                    point, q = value.pose.position, value.pose.orientation
                    norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
                    if norm < 1e-6:
                        raise ValueError('Invalid zero quaternion')
                    angle = euler_from_quaternion([q.x/norm, q.y/norm, q.z/norm, q.w/norm])[2]
                else:
                    value = message if message.header.frame_id == frame else listener.transformPoint(frame, message)
                    point, angle = value.point, yaw
                name, count = append_waypoint(filename, frame, dict(
                    x=round(float(point.x), 4), y=round(float(point.y), 4),
                    z=round(float(point.z), 4), yaw=round(float(angle), 4), wait_s=wait))
                rospy.loginfo('Recorded %s (#%d): x=%.3f y=%.3f z=%.3f yaw=%.3f -> %s',
                              name, count, point.x, point.y, point.z, angle, filename)
                status.publish(json.dumps(dict(state='SAVED', name=name, count=count,
                    x=point.x, y=point.y, z=point.z, yaw=angle, file=filename)))
        except (ValueError, TypeError, KeyError, OSError, yaml.YAMLError, tf.Exception) as exc:
            rospy.logerr('Waypoint NOT SAVED: %s; file=%s (existing coordinates preserved)', exc, filename)
            status.publish(json.dumps(dict(state='ERROR', error=str(exc), file=filename)))

    rospy.Subscriber(rospy.get_param('~clicked_topic', '/clicked_point'), PointStamped,
                     lambda msg: record(msg, False), queue_size=100)
    rospy.Subscriber(rospy.get_param('~pose_topic', '/slicer/patrol/record_goal'), PoseStamped,
                     lambda msg: record(msg, True), queue_size=100)
    rospy.loginfo('Waypoint recorder ready: %s; Publish Point yaw=%.3f rad; 2D Nav Goal records yaw.', filename, yaw)
    rospy.loginfo('Each successful click prints Recorded <name> (#count), x/y/z/yaw AFTER saving. Ctrl+C stops viewing/recording; no navigation goals are sent.')
    status.publish(json.dumps(dict(state='READY', file=filename)))
    rospy.spin()


if __name__ == '__main__':
    main()
