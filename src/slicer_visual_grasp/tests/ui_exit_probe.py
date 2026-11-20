"""仅供UI回归：假相机、禁止硬件，验证SIGINT/关闭窗口/强制退出。"""
import os
from pathlib import Path
import time
from unittest.mock import patch
import numpy as np
from slicer_visual_grasp import app
from slicer_visual_grasp.logic import Observation


class FakeCamera:
    def __init__(self,*args):pass
    def start(self):pass
    def snapshot(self):
        return np.zeros((480,640,3),np.uint8),Observation(1,time.monotonic(),None),(5,5,30,30),'simulated camera disconnect'
    def close(self,wait=True):
        if wait:
            raise AssertionError('退出不应阻塞等待相机')


class ProbeApp(app.App):
    def __init__(self,*args,**kwargs):
        super().__init__(*args,**kwargs)
        mode=os.environ.get('VG_PROBE_MODE','signal')
        if mode in ('busy','busy_x'):
            self.busy=True
        Path(os.environ['VG_PROBE_READY']).write_text('ready')
        if mode in ('x','busy_x'):
            self.win.after(150,self.close)


def forbidden(*args,**kwargs):
    raise AssertionError('退出测试禁止打开执行器')


if __name__=='__main__':
    with patch.object(app,'Camera',FakeCamera),patch.object(app,'App',ProbeApp),\
            patch('slicer_visual_grasp.devices.FeedbackChassis',forbidden),patch('slicer_visual_grasp.devices.AirGrip',forbidden):
        app.main()
