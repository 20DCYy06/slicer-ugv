#!/usr/bin/env python3
"""低频、固定 ID 的原点/车体标记，不累计轨迹或历史箭头。"""
import yaml
import rospy
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray


def make_markers(cfg):
    frame, base = cfg['frames']['map'], cfg['frames']['base']
    length, width = cfg['chassis']['body_length'], cfg['chassis']['body_width']
    result = MarkerArray()
    def marker(index, kind, reference, color):
        m = Marker()
        m.header.frame_id = reference
        m.ns, m.id, m.type, m.action = 'slicer_robot', index, kind, Marker.ADD
        m.pose.orientation.w = 1
        m.frame_locked = True
        m.color.r, m.color.g, m.color.b, m.color.a = color
        result.markers.append(m)
        return m
    body = marker(0, Marker.LINE_STRIP, base, (.1,.65,1.,1.))
    body.scale.x = .015
    body.points = [Point(x,y,.025) for x,y in [(-length/2,-width/2),(-length/2,width/2),
                   (length/2,width/2),(length/2,-width/2),(-length/2,-width/2)]]
    arrow = marker(1, Marker.ARROW, base, (.1,1.,.1,1.))
    arrow.pose.position.z = .055
    arrow.scale.x, arrow.scale.y, arrow.scale.z = .4,.07,.07
    origin = marker(2, Marker.SPHERE, frame, (1.,.8,0.,1.))
    origin.pose.position.z = .03
    origin.scale.x = origin.scale.y = origin.scale.z = .075
    label = marker(3, Marker.TEXT_VIEW_FACING, frame, (1.,.8,0.,1.))
    label.pose.position.y, label.pose.position.z = -.25,.12
    label.scale.z, label.text = .13,'MAP ORIGIN (0, 0)'
    return result


def main():
    rospy.init_node('slicer_navigation_visuals')
    with open(rospy.get_param('~config_file'), encoding='utf-8') as handle:
        cfg = yaml.safe_load(handle)
    publisher = rospy.Publisher('/slicer/navigation_markers', MarkerArray, queue_size=1, latch=True)
    markers = make_markers(cfg)
    # RViz can subscribe before AMCL establishes map->base_link.  One latched
    # message then stays in RViz's transform queue and the robot marker never
    # appears.  Re-publish the same fixed IDs at 1 Hz: no history is accumulated.
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        publisher.publish(markers)
        rate.sleep()


if __name__ == '__main__':
    main()
