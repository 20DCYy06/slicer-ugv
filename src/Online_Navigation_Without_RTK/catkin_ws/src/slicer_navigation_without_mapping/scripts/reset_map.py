#!/usr/bin/env python3
"""按命令重建在线地图；不重启传感器、定位、move_base、底盘或容器。"""
import time
import xmlrpc.client
import actionlib
import rosnode
import rospy
from move_base_msgs.msg import MoveBaseAction
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Empty as ResetNotice
from std_srvs.srv import Empty


def node_pid(name):
    code,_,uri=rospy.get_master().lookupNode(name)
    if code!=1:return None
    try:
        code,_,pid=xmlrpc.client.ServerProxy(uri).getPid(rospy.get_name())
        return pid if code==1 else None
    except (OSError,xmlrpc.client.Error):return None


def reset_map():
    old_pid=node_pid('/slam_gmapping')
    if old_pid is None:raise RuntimeError('slam_gmapping未运行；请先启动在线导航')
    notice=rospy.Publisher('/online_navigation/map_reset',ResetNotice,queue_size=1)
    client=actionlib.SimpleActionClient('/move_base',MoveBaseAction)
    if not client.wait_for_server(rospy.Duration(5.)):
        raise RuntimeError('move_base未就绪；未执行地图重置')
    # 清地图属于显式的新测试：停止旧巡航并取消旧目标，不沿新地图自动重发。
    if '/slicer_multi_goal_patrol' in rosnode.get_node_names():
        rosnode.kill_nodes(['/slicer_multi_goal_patrol'])
    client.cancel_all_goals()
    rospy.sleep(.2)
    started=rospy.Time.now()
    rospy.loginfo('[MapReset] 已取消旧目标，仅重新启动Gmapping，原PID=%s',old_pid)
    killed,failed=rosnode.kill_nodes(['/slam_gmapping'])
    if failed or not killed:raise RuntimeError('未能重启slam_gmapping')
    # roslaunch原生respawn重新启动同名SLAM，参数完全沿用原配置。
    # 这里只等待操作完成，不向导航/速度链添加任何超时或过滤条件。
    deadline=time.monotonic()+20.
    new_pid=None
    while time.monotonic()<deadline and not rospy.is_shutdown():
        new_pid=node_pid('/slam_gmapping')
        if new_pid is not None and new_pid!=old_pid:break
        rospy.sleep(.1)
    else:raise RuntimeError('Gmapping未重新启动；请查看导航日志')
    fresh_map=None
    while time.monotonic()<deadline and not rospy.is_shutdown():
        remaining=max(.1,deadline-time.monotonic())
        try:message=rospy.wait_for_message('/map',OccupancyGrid,timeout=remaining)
        except rospy.ROSException:break
        if message.header.stamp>=started and message.data:
            fresh_map=message;break
    if fresh_map is None:
        raise RuntimeError('Gmapping已重启，但未收到新地图；请检查激光扫描/TF')
    rospy.wait_for_service('/move_base/clear_costmaps',timeout=5.)
    rospy.ServiceProxy('/move_base/clear_costmaps',Empty)()
    notice.publish(ResetNotice());rospy.sleep(.1)
    rospy.loginfo('[MapReset] 地图已重置，SLAM PID=%s -> %s；可重新下发目标',old_pid,new_pid)


if __name__=='__main__':
    rospy.init_node('online_reset_map',anonymous=True)
    try:reset_map()
    except (RuntimeError,rospy.ROSException,rospy.ServiceException) as exc:
        rospy.logerr('[MapReset] %s',exc);raise SystemExit(1)
