#!/usr/bin/env python3
"""只读速度链/传感器诊断；不发布话题、不清地图、不发送或取消目标。"""
import argparse,json,time,threading
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan,Image,PointCloud2
from nav_msgs.msg import Path,Odometry
from actionlib_msgs.msg import GoalStatusArray
from std_msgs.msg import String,Bool

p=argparse.ArgumentParser(description=__doc__);p.add_argument('--duration',type=float,default=30);a=p.parse_args()
rospy.init_node('online_readonly_diagnosis',anonymous=True);samples={};lock=threading.Lock()
def receive(m,topic):
    value=None
    if isinstance(m,Twist):value=[round(m.linear.x,3),round(m.angular.z,3)]
    elif isinstance(m,Path):value=len(m.poses)
    elif isinstance(m,(String,Bool)):value=m.data
    elif isinstance(m,Odometry):value=[round(m.twist.twist.linear.x,3),round(m.twist.twist.angular.z,3)]
    elif isinstance(m,GoalStatusArray):value=[{'state':s.status,'text':s.text} for s in m.status_list[-1:]]
    stamp=m.header.stamp.to_sec() if hasattr(m,'header') and m.header.stamp else None
    with lock:samples[topic]=(time.monotonic(),stamp,value)
for topic,typ in [('/cmd_vel/nav_raw',Twist),('/cmd_vel/nav',Twist),('/cmd_vel',Twist),('/wheel/odom',Odometry),
    ('/Odometry',Odometry),('/scan',LaserScan),
    ('/move_base/TebLocalPlannerROS/local_plan',Path),('/move_base/status',GoalStatusArray),
    ('/slicer_nav_arc_guard/hold_reason',String),('/slicer_c30d_driver/ready',Bool)]:
    rospy.Subscriber(topic,typ,receive,callback_args=topic,queue_size=1,buff_size=2**21)
print('READ_ONLY: 速度=[vx,wz]；rx_age=接收间隔，stamp_age=消息时间戳年龄（秒）。',flush=True)
end=time.monotonic()+a.duration
while not rospy.is_shutdown() and time.monotonic()<end:
    time.sleep(1);now=time.monotonic();ros_now=rospy.Time.now().to_sec()
    with lock:latest=dict(samples)
    print(json.dumps({k:{'rx_age':round(now-v[0],3),'stamp_age':round(ros_now-v[1],3) if v[1] else None,'value':v[2]} for k,v in latest.items()},ensure_ascii=False),flush=True)
