#!/usr/bin/env python3
"""真实Gmapping+move_base+双层costmap的隔离测试；发布的目标只进入无设备测试容器。"""
from pathlib import Path
import os,signal,subprocess,time,sys,xmlrpc.client
import rospy,actionlib,rosnode
from nav_msgs.msg import OccupancyGrid,Path as RosPath
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction,MoveBaseGoal

PACKAGE=Path(__file__).resolve().parents[1]
runtime=sys.argv[1];processes=[];streams=[]
def start(args,name):
    stream=open('/tmp/'+name+'.log','w');streams.append(stream)
    p=subprocess.Popen(args,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True);processes.append(p);return p
def wait(predicate,seconds=35):
    end=time.monotonic()+seconds
    while time.monotonic()<end and not rospy.is_shutdown():
        if predicate():return
        time.sleep(.05)
    raise AssertionError('Timed out: inspect /tmp/online-launch.log and /tmp/online-mock.log')
def patch_cost(grid,x,y):
    if grid is None:return -1
    ox,oy=grid.info.origin.position.x,grid.info.origin.position.y;r=grid.info.resolution
    ix,iy=int((x-ox)/r),int((y-oy)/r)
    return max([grid.data[yy*grid.info.width+xx] for yy in range(iy-1,iy+2) for xx in range(ix-1,ix+2) if 0<=xx<grid.info.width and 0<=yy<grid.info.height] or [-1])
try:
    start(['roslaunch',str(PACKAGE/'launch/slicer_nav_without_mapping.launch'),'runtime_root:='+runtime,'start_sensors:=false','start_control:=true','control_mock:=true','gui:=false'],'online-launch')
    start(['python3',str(PACKAGE/'tests/mock_sensors.py')],'online-mock')
    rospy.init_node('online_integration',anonymous=True);state={'grid':None,'map':None,'plan':0,'raw':False,'last_raw':(0.,0.),'output':(0.,0.)}
    rospy.Subscriber('/cmd_vel',Twist,lambda m:state.update(output=(m.linear.x,m.angular.z)),queue_size=1)
    rospy.Subscriber('/map',OccupancyGrid,lambda m:state.update(map=m),queue_size=1)
    rospy.Subscriber('/move_base/local_costmap/costmap',OccupancyGrid,lambda m:state.update(grid=m),queue_size=1)
    rospy.Subscriber('/move_base/TebLocalPlannerROS/local_plan',RosPath,lambda m:state.update(plan=len(m.poses)),queue_size=1)
    rospy.Subscriber('/cmd_vel/nav_raw',Twist,lambda m:state.update(raw=state['raw'] or abs(m.linear.x)+abs(m.angular.z)>.001,last_raw=(m.linear.x,m.angular.z)),queue_size=1)
    wait(lambda:state['map'] is not None and state['grid'] is not None)
    import yaml
    vehicle=yaml.safe_load(Path('/slicer/config/slicer_config.yaml').read_text())['navigation']
    assert rospy.get_param('/move_base/TebLocalPlannerROS/max_vel_x')==vehicle['max_forward']
    assert rospy.get_param('/move_base/TebLocalPlannerROS/max_vel_theta')==1.2
    assert rospy.get_param('/move_base/TebLocalPlannerROS/weight_acc_lim_theta')==3.0
    assert rospy.get_param('/move_base/TebLocalPlannerROS/min_turning_radius')==0.
    nodes=rosnode.get_node_names();assert '/slam_gmapping' in nodes and '/amcl' not in nodes and '/map_server' not in nodes
    wait(lambda:patch_cost(state['grid'],.74,0)>=99)
    assert patch_cost(state['grid'],.9,.5)>=99,'Lidar obstacle missing'
    client=actionlib.SimpleActionClient('/move_base',MoveBaseAction);assert client.wait_for_server(rospy.Duration(15))
    goal=MoveBaseGoal();goal.target_pose.header.frame_id='map';goal.target_pose.header.stamp=rospy.Time.now();goal.target_pose.pose.position.x=1.4;goal.target_pose.pose.orientation.w=1
    client.send_goal(goal);wait(lambda:state['plan']>0 and state['raw'],20)
    wait(lambda:sum(abs(v) for v in state['output'])>.001,5)
    client.cancel_goal()
    rospy.set_param('/online_test/lidar_box',False)
    wait(lambda:0<=patch_cost(state['grid'],.74,0)<99,15)
    assert patch_cost(state['grid'],.9,.5)>=99,'Static lidar obstacle unexpectedly disappeared'
    # 只在无设备的隔离容器执行reset-map：真正重启SLAM，其他进程及参数不变。
    def pid(name):
        code,_,uri=rospy.get_master().lookupNode(name)
        assert code==1,name
        return xmlrpc.client.ServerProxy(uri).getPid('/online_integration')[2]
    pids={name:pid(name) for name in ['/slam_gmapping','/move_base','/online_mock_sensors']}
    slam_params=rospy.get_param('/slam_gmapping')
    client.send_goal(goal);wait(lambda:client.get_state()==1)
    subprocess.run([sys.executable,str(PACKAGE/'scripts/reset_map.py')],check=True,timeout=30)
    assert pid('/slam_gmapping')!=pids['/slam_gmapping'],'SLAM did not actually restart'
    assert pid('/move_base')==pids['/move_base'],'Reset must not restart planner'
    assert pid('/online_mock_sensors')==pids['/online_mock_sensors'],'Reset must not restart sensors'
    assert rospy.get_param('/slam_gmapping')==slam_params,'Reset changed SLAM parameters'
    wait(lambda:client.get_state()==2)
    client.send_goal(goal)
    wait(lambda:client.get_state()==1 and sum(abs(v) for v in state['last_raw'])>.001,15)
    client.cancel_goal()
    print('ONLINE_MAP_RESET_PASS: SLAM restarted; planner/sensors/parameters retained; old goal canceled; next goal runs',flush=True)
    print('ONLINE_INTEGRATION_PASS: blank startup -> real SLAM/map -> B-spline/TEB path -> guard/mux nonzero; lidar marking/clearing; mock Workspace2 driver; no camera',flush=True)
finally:
    for p in reversed(processes):
        if p.poll() is None:
            os.killpg(p.pid,signal.SIGINT)
            try:p.wait(timeout=8)
            except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()
    for s in streams:s.close()
