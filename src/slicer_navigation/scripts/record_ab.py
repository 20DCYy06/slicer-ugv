#!/usr/bin/env python3
"""在专用RViz目标话题记录A、B，绝不触发move_base。原子写入配置并留备份。"""
import datetime
import math
import os
from pathlib import Path
import tempfile
import yaml
import rospy
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion


def main():
    rospy.init_node('slicer_record_ab');path=Path(rospy.get_param('~config_file')).resolve()
    cfg=yaml.safe_load(path.read_text());keys=['A','B'];index=[0]
    overwrite=rospy.get_param('~overwrite',False)
    if not overwrite:keys=[k for k in keys if cfg['navigation'][k] is None]
    if not keys:
        rospy.logerr('A/B均存在；需显式 overwrite:=true 才能重录');return
    def callback(msg):
        if index[0]>=len(keys):return
        if msg.header.frame_id!=cfg['frames']['map']:
            rospy.logerr('仅接受map坐标，拒绝%s',msg.header.frame_id);return
        q=msg.pose.orientation;values=[q.x,q.y,q.z,q.w,msg.pose.position.x,msg.pose.position.y]
        if not all(math.isfinite(x) for x in values) or abs(sum(x*x for x in values[:4])-1)>.01:
            rospy.logerr('无效坐标或四元数');return
        key=keys[index[0]]
        current=yaml.safe_load(path.read_text())
        current['navigation'][key]={'x':float(msg.pose.position.x),'y':float(msg.pose.position.y),
                                     'yaw':float(euler_from_quaternion(values[:4])[2])}
        backup=path.with_name(path.name+'.before_ab_'+datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
        backup.write_bytes(path.read_bytes())
        tmp=None
        try:
            with tempfile.NamedTemporaryFile('w',dir=path.parent,delete=False,encoding='utf-8') as f:
                tmp=f.name;yaml.safe_dump(current,f,sort_keys=False,allow_unicode=True);f.flush();os.fsync(f.fileno())
            os.replace(tmp,str(path))
        finally:
            if tmp and os.path.exists(tmp):os.unlink(tmp)
        rospy.loginfo('Recorded %s=%s',key,current['navigation'][key]);index[0]+=1
        if index[0]==len(keys):rospy.loginfo('A/B记录完成，不会发送任何运动目标')
    sub=rospy.Subscriber('/slicer/record_goal',PoseStamped,callback,queue_size=1)
    rospy.loginfo('RViz: 2D Nav Goal 的 Topic 选 /slicer/record_goal；依次 A、B')
    rospy.spin()


if __name__=='__main__':main()
