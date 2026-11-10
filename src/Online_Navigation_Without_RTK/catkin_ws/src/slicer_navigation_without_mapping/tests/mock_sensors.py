#!/usr/bin/env python3
"""仅network=none隔离检查使用：合成二维扫描、里程计和TF，无硬件。"""
import rospy,tf,numpy as np
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
rospy.init_node('online_mock_sensors')
scan_pub=rospy.Publisher('/scan',LaserScan,queue_size=1);odom_pub=rospy.Publisher('/odom',Odometry,queue_size=1)
br=tf.TransformBroadcaster();rate=rospy.Rate(10)
while not rospy.is_shutdown():
    stamp=rospy.Time.now();br.sendTransform((0,0,0),(0,0,0,1),stamp,'base_link','odom')
    odom=Odometry();odom.header.stamp=stamp;odom.header.frame_id='odom';odom.child_frame_id='base_link';odom.pose.pose.orientation.w=1;odom_pub.publish(odom)
    scan=LaserScan();scan.header.stamp=stamp;scan.header.frame_id='base_link';scan.angle_min=-np.pi;scan.angle_increment=2*np.pi/720;scan.angle_max=scan.angle_min+719*scan.angle_increment
    scan.range_min=.05;scan.range_max=20;scan.scan_time=.1
    theta=scan.angle_min+np.arange(720)*scan.angle_increment;dx,dy=np.cos(theta),np.sin(theta)
    ranges=3./np.maximum(abs(dx),abs(dy))
    obstacles=[(.9,.5,.08)]
    if rospy.get_param('/online_test/lidar_box',True):obstacles.append((.74,0.,.07))
    for x,y,radius in obstacles:
        projection=x*dx+y*dy;disc=projection**2-(x*x+y*y-radius*radius)
        hit=(disc>0)&(projection>0);ranges[hit]=np.minimum(ranges[hit],projection[hit]-np.sqrt(disc[hit]))
    scan.ranges=ranges.tolist();scan_pub.publish(scan);rate.sleep()
