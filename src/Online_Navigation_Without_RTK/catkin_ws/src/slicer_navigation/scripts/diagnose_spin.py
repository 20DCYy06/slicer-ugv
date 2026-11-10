#!/usr/bin/env python3
"""只读巡航自转诊断：不发布速度、不打开串口。"""
import argparse
import math
import sys
import threading
import time

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Log
from std_msgs.msg import Bool, String
from tf.transformations import euler_from_quaternion


def yaw(orientation):
    q = orientation
    return euler_from_quaternion((q.x, q.y, q.z, q.w))[2]


class SpinMonitor:
    def __init__(self):
        self.lock = threading.Lock()
        self.samples = {}
        self.stage = 'WAITING'
        self.spin_started = None
        self.max_abs = {key: 0.0 for key in ('raw', 'guard', 'motor', 'wheel')}
        self.first_yaw = None
        self.last_yaw = None
        self.guard_warning = ''
        self.subscribers = [
            rospy.Subscriber('/cmd_vel/patrol_spin_raw', Twist,
                             lambda msg: self.record('raw', msg.angular.z), queue_size=10),
            rospy.Subscriber('/cmd_vel/nav', Twist,
                             lambda msg: self.record('guard', msg.angular.z), queue_size=10),
            rospy.Subscriber('/cmd_vel', Twist,
                             lambda msg: self.record('motor', msg.angular.z), queue_size=10),
            rospy.Subscriber('/wheel/odom', Odometry,
                             lambda msg: self.record('wheel', msg.twist.twist.angular.z), queue_size=10),
            rospy.Subscriber('/lio/odom', Odometry,
                             lambda msg: self.record('lio_yaw', yaw(msg.pose.pose.orientation)), queue_size=10),
            rospy.Subscriber('/odom', Odometry,
                             lambda msg: self.record('fused_yaw', yaw(msg.pose.pose.orientation)), queue_size=10),
            rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped,
                             self.on_amcl, queue_size=10),
            rospy.Subscriber('/slicer_c30d_driver/ready', Bool,
                             lambda msg: self.record('ready', msg.data), queue_size=10),
            rospy.Subscriber('/slicer/localization_ready', Bool,
                             lambda msg: self.record('localized', msg.data), queue_size=10),
            rospy.Subscriber('/slicer_multi_goal_patrol/state', String, self.on_stage, queue_size=10),
            rospy.Subscriber('/rosout', Log, self.on_log, queue_size=10),
        ]

    def on_amcl(self, msg):
        self.record('amcl', yaw(msg.pose.pose.orientation))
        cov = msg.pose.covariance
        self.record('amcl_cov_xyyaw', tuple(round(cov[i], 4) for i in (0, 7, 35)))

    def record(self, key, value):
        now = time.monotonic()
        with self.lock:
            self.samples[key] = (now, value)
            if self.spin_started is not None:
                if key in self.max_abs:
                    self.max_abs[key] = max(self.max_abs[key], abs(value))
                if key == 'amcl':
                    if self.first_yaw is None:
                        self.first_yaw = value
                    self.last_yaw = value

    def on_stage(self, msg):
        with self.lock:
            previous = self.stage
            self.stage = msg.data
            if msg.data.startswith('SPIN_'):
                self.spin_started = time.monotonic()
                self.max_abs = {key: 0.0 for key in self.max_abs}
                self.first_yaw = self.last_yaw = None
                self.guard_warning = ''
            elif previous.startswith('SPIN_'):
                self.summary(previous)
                self.spin_started = None
        print('STAGE %s' % msg.data, flush=True)

    def on_log(self, msg):
        if msg.name == '/slicer_nav_arc_guard' and msg.level >= Log.WARN:
            with self.lock:
                self.guard_warning = msg.msg
            print('GUARD_WARNING %s' % msg.msg, flush=True)

    def summary(self, stage):
        values = self.max_abs
        change = 'unknown' if self.first_yaw is None or self.last_yaw is None else \
            '%.3f' % math.atan2(math.sin(self.last_yaw - self.first_yaw),
                                math.cos(self.last_yaw - self.first_yaw))
        print('SUMMARY %s max_abs_w(raw/guard/motor/wheel)=%s amcl_yaw_change=%srad' %
              (stage, {key: round(value, 3) for key, value in values.items()}, change), flush=True)
        if values['raw'] < 0.05:
            print('CHECK 巡航节点没有持续发出非零自转指令。', flush=True)
        elif values['guard'] < 0.05:
            print('CHECK 指令在避障/速度出口被归零；查看 GUARD_WARNING。', flush=True)
        elif values['motor'] < 0.05:
            print('CHECK 指令在速度仲裁到电机入口之间被归零。', flush=True)
        elif values['wheel'] < 0.03:
            print('CHECK 电机入口有指令但编码器未见自转；检查 PWM 起转力、使能和底层实际输出。', flush=True)
        elif change == 'unknown' or abs(float(change)) < 0.05:
            print('CHECK 轮子有转动而 AMCL 朝向未更新；检查打滑/定位。', flush=True)
        else:
            print('CHECK 电机与朝向均有反馈；核对目标 yaw、角度误差与超时时间。', flush=True)
        if self.guard_warning:
            print('LAST_GUARD_WARNING %s' % self.guard_warning, flush=True)

    def display(self):
        now = time.monotonic()
        with self.lock:
            if self.spin_started is None:
                return
            fields = []
            for key in ('raw', 'guard', 'motor', 'wheel', 'lio_yaw', 'fused_yaw',
                        'amcl', 'amcl_cov_xyyaw', 'ready', 'localized'):
                sample = self.samples.get(key)
                if sample is None:
                    fields.append('%s=NO_DATA' % key)
                else:
                    age, value = now - sample[0], sample[1]
                    shown = '%.3f' % value if isinstance(value, float) else str(value)
                    fields.append('%s=%s(%.1fs)' % (key, shown, age))
            stage = self.stage
        print('%s %s' % (stage, ' '.join(fields)), flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=90.0,
                        help='最长监听秒数；Ctrl+C 可以提前结束')
    args = parser.parse_args(rospy.myargv(argv=sys.argv)[1:])
    if args.duration <= 0:
        parser.error('--duration must be positive')
    rospy.init_node('slicer_spin_diagnostic', anonymous=True)
    monitor = SpinMonitor()
    print('只读监听已就绪；在另一终端启动巡航。这里不发送任何电机指令。', flush=True)
    deadline = time.monotonic() + args.duration
    while not rospy.is_shutdown() and time.monotonic() < deadline:
        monitor.display()
        time.sleep(0.5)
    with monitor.lock:
        if monitor.spin_started is not None:
            monitor.summary(monitor.stage)


if __name__ == '__main__':
    try:
        main()
    except (KeyboardInterrupt, rospy.ROSInterruptException):
        pass
