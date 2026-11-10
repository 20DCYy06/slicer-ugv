#!/usr/bin/env python3
"""定位就绪状态带超时；初始位姿与全局重定位显式选择。"""
import math
import threading
import time
import yaml
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Bool
from std_msgs.msg import String
from std_srvs.srv import Empty
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
import tf
from tf.transformations import quaternion_from_euler


def main():
    rospy.init_node('slicer_amcl_ready')
    with open(rospy.get_param('~config_file'), encoding='utf-8') as handle:
        cfg = yaml.safe_load(handle)
    n = cfg['navigation']
    pub = rospy.Publisher('/slicer/localization_ready', Bool, latch=True, queue_size=1)
    pub.publish(False)
    status = rospy.Publisher('~state', String, latch=True, queue_size=1)
    initialization = rospy.Publisher('~initialization', String, latch=True, queue_size=1)
    initialization.publish('WAITING')
    status.publish('INITIALIZING')
    lock = threading.Lock()
    state = {'at': -math.inf, 'good': False, 'samples': 0, 'manual': False,
             'cov': None, 'reason': 'no_pose'}

    def callback(msg):
        c, q = msg.pose.covariance, msg.pose.pose.orientation
        values = [c[0], c[7], c[35], q.x, q.y, q.z, q.w,
                  msg.pose.pose.position.x, msg.pose.pose.position.y]
        finite = all(math.isfinite(v) for v in values) and sum(v*v for v in (q.x,q.y,q.z,q.w)) > 0
        if not finite:
            reason = 'invalid_pose'
        elif min(c[0], c[7], c[35]) < 0:
            reason = 'negative_covariance'
        elif max(c[0], c[7]) >= n['amcl_xy_variance']:
            reason = 'xy_covariance'
        elif c[35] >= n['amcl_yaw_variance']:
            reason = 'yaw_covariance'
        else:
            reason = 'within_limits'
        good = reason == 'within_limits'
        with lock:
            state['samples'] = state['samples'] + 1 if good else 0
            state['good'] = state['samples'] >= 2
            state['at'] = time.monotonic()
            state['cov'] = (c[0], c[7], c[35])
            state['reason'] = reason

    sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, callback, queue_size=1)
    def initial_pose_seen(unused):
        with lock:
            state['manual'] = True
            state['samples'], state['good'] = 0, False
    initial_sub = rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, initial_pose_seen, queue_size=1)
    set_pose = rospy.get_param('~set_initial_pose', False)
    global_search = rospy.get_param('~global_localization', False)
    if set_pose and global_search:
        raise ValueError('Choose set_initial_pose OR global_localization')
    def wait_message(topic, message_type):
        while not rospy.is_shutdown():
            try:
                return rospy.wait_for_message(topic, message_type, timeout=5)
            except rospy.ROSException:
                rospy.logwarn_throttle(5, 'Automatic AMCL waiting for %s', topic)
        raise rospy.ROSInterruptException('shutdown')
    if set_pose:
        pose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, latch=True, queue_size=1)
        deadline = time.monotonic() + 10
        while pose_pub.get_num_connections() == 0 and not rospy.is_shutdown():
            if time.monotonic() > deadline:
                raise RuntimeError('AMCL did not subscribe to /initialpose')
            time.sleep(0.05)
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id, msg.header.stamp = cfg['frames']['map'], rospy.Time.now()
        msg.pose.pose.position.x = float(rospy.get_param('~initial_x'))
        msg.pose.pose.position.y = float(rospy.get_param('~initial_y'))
        q = quaternion_from_euler(0, 0, float(rospy.get_param('~initial_yaw')))
        msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = q
        msg.pose.covariance[0] = msg.pose.covariance[7] = 0.04
        msg.pose.covariance[35] = 0.09
        pose_pub.publish(msg)
        initialization.publish('PARAM_INITIAL_POSE')
        status.publish('INITIAL_POSE_SENT')
        rospy.loginfo('Initial pose published; waiting for AMCL convergence')
    elif global_search:
        status.publish('WAITING_FOR_MAP_AND_SCAN')
        # 服务可能先于地图加载就出现；先等地图、实际激光及 odom TF 都已接通。
        wait_message('/map', OccupancyGrid)
        wait_message('/scan', LaserScan)
        listener = tf.TransformListener()
        while not rospy.is_shutdown():
            try:
                listener.waitForTransform(cfg['frames']['odom'], cfg['frames']['base'], rospy.Time(0), rospy.Duration(3))
                break
            except tf.Exception:
                rospy.logwarn_throttle(5, 'Automatic AMCL waiting for odom -> base TF')
        rospy.wait_for_service('/global_localization', timeout=30)
        # 等 AMCL 自己处理首帧激光，确认内部地图已安装，避免服务提前返回而未撒点。
        request = rospy.ServiceProxy('/request_nomotion_update', Empty)
        rospy.wait_for_service('/request_nomotion_update', timeout=30)
        request()
        wait_message('/amcl_pose', PoseWithCovarianceStamped)
        with lock:
            manual = state['manual']
        if not manual:
            rospy.ServiceProxy('/global_localization', Empty)()
            with lock:
                state.update(at=-math.inf, good=False, samples=0)
            status.publish('AUTO_LOCALIZING')
            initialization.publish('AUTO_GLOBAL')
            rospy.loginfo('Automatic AMCL global localization requested; waiting for measured convergence')
            request()
        else:
            status.publish('MANUAL_INITIAL_POSE')
            initialization.publish('MANUAL_INITIAL_POSE')
            rospy.loginfo('Manual initial pose received; preserving it instead of global reset')
    else:
        initialization.publish('MANUAL_REQUIRED')
        rospy.loginfo('AMCL monitor ready; use RViz 2D Pose Estimate for initial localization')
    # AMCL 位姿按激光更新条件发布；静止时不应沿用极短超时。
    timeout = rospy.get_param('~pose_timeout', 10.0)
    nomotion = rospy.ServiceProxy('/request_nomotion_update', Empty)
    next_update = 0.0
    previous = None
    while not rospy.is_shutdown():
        with lock:
            good = state['good'] and time.monotonic() - state['at'] <= timeout
            age = time.monotonic() - state['at']
            cov, reason, samples = state['cov'], state['reason'], state['samples']
        if (not good or age > 2.0) and time.monotonic() >= next_update:
            try:
                nomotion()
            except rospy.ServiceException:
                pass
            next_update = time.monotonic() + (1.0 if not good else 2.0)
        pub.publish(good)
        if previous is None or good != previous:
            status.publish('READY' if good else 'LOCALIZING')
            rospy.loginfo('AMCL %s: reason=%s age=%.2fs samples=%d cov_xyyaw=%s limits=(%.4f,%.4f)',
                          'READY' if good else 'LOCALIZING',
                          'stale_pose' if age > timeout else reason, age, samples, cov,
                          n['amcl_xy_variance'], n['amcl_yaw_variance'])
            previous = good
        time.sleep(0.25)


if __name__ == '__main__':
    try:
        main()
    except (rospy.ROSInterruptException, KeyboardInterrupt):
        pass
