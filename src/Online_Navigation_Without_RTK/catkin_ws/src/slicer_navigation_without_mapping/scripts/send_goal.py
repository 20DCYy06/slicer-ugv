#!/usr/bin/env python3
"""发送一个ROS1 MoveBase目标；--relative使用当前底盘坐标，yaw单位rad。"""
import argparse,math
import actionlib,rospy,tf
from move_base_msgs.msg import MoveBaseAction,MoveBaseGoal
from tf.transformations import quaternion_from_euler

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--x',type=float,required=True);p.add_argument('--y',type=float,default=0.)
    p.add_argument('--yaw',type=float,default=0.);p.add_argument('--relative',action='store_true');a=p.parse_args()
    if not all(math.isfinite(v) for v in (a.x,a.y,a.yaw)):p.error('坐标必须有限')
    rospy.init_node('online_send_goal',anonymous=True);client=actionlib.SimpleActionClient('/move_base',MoveBaseAction)
    if not client.wait_for_server(rospy.Duration(10)):raise RuntimeError('move_base尚未就绪')
    goal=MoveBaseGoal();pose=goal.target_pose;pose.header.frame_id='base_link' if a.relative else 'map'
    pose.pose.position.x=a.x;pose.pose.position.y=a.y
    q=quaternion_from_euler(0,0,a.yaw);pose.pose.orientation.x,pose.pose.orientation.y,pose.pose.orientation.z,pose.pose.orientation.w=q
    if a.relative:
        listener=tf.TransformListener();listener.waitForTransform('map','base_link',rospy.Time(0),rospy.Duration(5))
        goal.target_pose=listener.transformPose('map',pose)
    goal.target_pose.header.stamp=rospy.Time.now();client.send_goal(goal);rospy.sleep(.3)
    rospy.loginfo('Goal sent in map: x=%.3f y=%.3f',goal.target_pose.pose.position.x,goal.target_pose.pose.position.y)
