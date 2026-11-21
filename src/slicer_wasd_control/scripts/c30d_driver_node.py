#!/usr/bin/env python3
"""唯一串口所有者：/cmd_vel -> Test同款0x16闭环；不产生开环PWM指令。"""
import copy,math,os,sys,time
from pathlib import Path
sys.dont_write_bytecode=True
sys.path.insert(0,str(Path(__file__).resolve().parent))
prefix=Path(__file__).resolve().parents[2]
workspace=Path(os.environ.get('WORKSPACE2_ROOT') or (prefix if (prefix/'Hardware/Host').is_dir()
                                                    else Path(__file__).resolve().parents[3]))
sys.path.insert(0,str(workspace/'Hardware/Host'))
from chassis import Chassis,load_config as load_host
from chassis_commands import CommandServer
from control_core import load_config
from driver_core import Session,EncoderTap,Odometry
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry as OdomMsg
from std_msgs.msg import Bool

def main():
    rospy.init_node('slicer_c30d_driver')
    c=load_config(rospy.get_param('~config_file'))
    host=copy.deepcopy(load_host(rospy.get_param('~host_config_file',str(workspace/'Hardware/config.yaml'))))
    # 导航使用自身的速度参数，MCU 仍执行其编译限幅。
    host['speed']['linear_max']=c['max_linear'];host['speed']['angular_max']=c['max_angular']
    car=None;ready_pub=rospy.Publisher('~ready',Bool,queue_size=1,latch=True)
    odom_pub=rospy.Publisher(c['odom_topic'],OdomMsg,queue_size=5)
    ready_pub.publish(False);session=None;odom=Odometry(c)
    mock=rospy.get_param('~mock',False)
    test_service=None;commands=None
    def receive(msg):
        if session is not None:session.command(msg.linear.x,msg.angular.z,time.monotonic())
    rospy.Subscriber(c['output_topic'],Twist,receive,queue_size=1)
    try:
        if not mock:
            car=Chassis(host);tap=EncoderTap(car.parser);car.parser=tap;car.connect()
            session=Session(car,c,rospy.get_param('~wait_for_fault_clear',True))
            if session.wait_fault:
                commands=CommandServer(host['serial']['serial_number'],session.link_recovery)
                commands.start(workspace/'Hardware/Unlock/unlock.py')
            if session.wait_fault and os.environ.get('SLICER_C30D_ENABLE_LOCK_TEST')=='1':
                test_service=session.link_recovery.start_test_service(rospy)
            rospy.loginfo('C30D USB=%s signs=%s/%s; 0x16 vx/wz; PWM cap=%.1f%%; lidar-only navigation',
                          host['serial']['serial_number'],host['serial']['pwm_signs'],
                          host['serial']['encoder_signs'],host['control'].get('mcu_pwm_accept_max_permille',800)/10)
        else:rospy.loginfo('MOCK_ONLY: no serial is opened, no physical motor commands')
        while not rospy.is_shutdown():
            value=None
            if session:
                v,w,ready,first,recovered=session.cycle(time.monotonic());ready_pub.publish(ready)
                if first:rospy.logerr('MCU_FAULT_HELD flags=%s; zero output during chassis recovery',session.fault_flags)
                if session.recovery_error:rospy.logwarn_throttle(3.,'MCU recovery pending: %s',session.recovery_error)
                if recovered:rospy.loginfo('MCU_RECOVERED: old velocity discarded; only fresh commands accepted')
                value=odom.update(tap.sample,car.body)
            else:
                ready_pub.publish(True);value=(0.,0.,0.,0.,0.)
            if value:
                x,y,yaw,v,w=value;msg=OdomMsg();msg.header.stamp=rospy.Time.now()
                msg.header.frame_id=c['odom_frame'];msg.child_frame_id=c['base_frame']
                msg.pose.pose.position.x=x;msg.pose.pose.position.y=y
                msg.pose.pose.orientation.z=math.sin(yaw/2);msg.pose.pose.orientation.w=math.cos(yaw/2)
                msg.twist.twist.linear.x=v;msg.twist.twist.angular.z=w
                for i in [0,7,35]:msg.pose.covariance[i]=msg.twist.covariance[i]=.1
                for i in [14,21,28]:msg.pose.covariance[i]=msg.twist.covariance[i]=1e6
                odom_pub.publish(msg)
            time.sleep(1./c['publish_hz'])
    finally:
        if commands:session.link_recovery.stop_requests()
        if test_service:test_service.shutdown()
        try:ready_pub.publish(False)
        except rospy.ROSException:pass
        try:
            if car:car.close()
        finally:
            if commands:commands.close()
if __name__=='__main__':
    try:main()
    except (KeyboardInterrupt,rospy.ROSInterruptException):pass
    except Exception as exc:rospy.logfatal('%s',exc);raise SystemExit(1)
