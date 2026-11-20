#!/usr/bin/env python3
"""唯一/cmd_vel出口：键盘 > 任务 > 导航；过期输入归零。"""
import math
import threading
import time
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
import rospy
from geometry_msgs.msg import Twist
from control_core import DEFAULT_CONFIG, load_config


def main():
    rospy.init_node('slicer_velocity_mux')
    c=load_config(rospy.get_param('~config_file',str(DEFAULT_CONFIG)))
    slots={};lock=threading.Lock()
    pub=rospy.Publisher(c['output_topic'],Twist,queue_size=1)
    def receive(msg,name):
        if not all(math.isfinite(x) for x in (msg.linear.x,msg.angular.z)):
            msg=Twist()
        with lock:slots[name]=(time.monotonic(),msg)
    subs=[rospy.Subscriber(c[name+'_topic'],Twist,receive,callback_args=name,queue_size=1)
          for name in ('teleop','mission','nav')]
    try:
        while not rospy.is_shutdown():
            now=time.monotonic();out=Twist()
            with lock:
                for name in ('teleop','mission','nav'):
                    if name in slots and now-slots[name][0]<c['command_timeout_s']:
                        out=slots[name][1];break
            pub.publish(out);time.sleep(1/c['publish_hz'])
    finally:
        try:pub.publish(Twist())
        except rospy.ROSException:pass


if __name__=='__main__':main()
