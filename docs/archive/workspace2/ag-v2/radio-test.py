"""Isolated FKIE probe. The fake backend has no ROS navigation interfaces."""
import json
import os
import pathlib
import runpy
import signal
import subprocess
import threading
import time
from xmlrpc.client import ServerProxy

import rospy
from std_msgs.msg import String
from ag_uav_client.v2 import UAV_STATE, vehicle_position
from ag_ugv_task.executor import TaskExecutor
from ag_ugv_task.node import GroundNode

assert os.environ['ROS_MASTER_URI'] == 'http://192.168.10.33:21311'
case = pathlib.Path('/ag_v2_ws/src/AG_Client/ag_ugv_task/test/check_v2_ros.py')
FakeNavigation = runpy.run_path(str(case))['FakeNavigation']

class RadioFake(FakeNavigation):
    def cancel(self):
        super().cancel()
        self.state = 2

launch = subprocess.Popen([
    'roslaunch','-p','21311','ag_uav_client','fkie_radio.launch',
    'local_name:=ugv-v2-probe','peer_name:=uav-v2-probe','peer_ip:=192.168.10.32',
    'discovery_port:=21511','rpc_port:=21611'], start_new_session=True)
try:
    master = ServerProxy(os.environ['ROS_MASTER_URI'])
    deadline = time.monotonic() + 10
    while time.monotonic() < deadline:
        try:
            if master.getPid('/ag_radio_probe')[0] == 1:
                break
        except OSError:
            pass
        if launch.poll() is not None:
            raise RuntimeError('UGV isolated FKIE stopped')
        time.sleep(.1)
    else:
        raise RuntimeError('No UGV isolated ROS master')
    rospy.init_node('ag_radio_ground_probe')
    fake = RadioFake()
    ground = GroundNode(fake, TaskExecutor, lambda: vehicle_position())
    received = threading.Event()

    def record(msg):
        data = json.loads(msg.data)
        last = data.get('last_command') or {}
        if data.get('vehicle_id') == 'uav' and last.get('task_id','').startswith('radio-probe-'):
            received.set()
            rospy.loginfo('UAV telemetry crossed FKIE: %s, %s', last['task_id'], last.get('op'))

    listener = rospy.Subscriber(UAV_STATE, String, record, queue_size=1)
    worker = threading.Thread(target=ground.run, daemon=True)
    worker.start()
    deadline = time.monotonic() + 110
    while time.monotonic() < deadline and not rospy.is_shutdown() and launch.poll() is None:
        if ground.route.phase == 'CANCELED' and received.is_set() and len(fake.goals) == 1:
            print('PASS UGV: one synthetic goal and STOP in fake executor; UAV state received', flush=True)
            time.sleep(2)
            break
        time.sleep(.1)
    else:
        raise RuntimeError('Timed out waiting for isolated UAV test')
finally:
    if rospy.core.is_initialized():
        rospy.signal_shutdown('test complete')
    if launch.poll() is None:
        os.killpg(launch.pid, signal.SIGINT)
        launch.wait(timeout=12)
