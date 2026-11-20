"""在Xvfb中使用假相机测试800×480界面；绝不连接串口或真实摄像头。"""
import tkinter as tk
from unittest.mock import patch
import numpy as np
import app
from logic import Observation


class FakeCamera:
    def __init__(self,*args):pass
    def start(self):pass
    def close(self,wait=True):pass
    def select(self,box):pass
    def snapshot(self):
        return np.zeros((480,640,3),np.uint8), Observation(1,1,(320,240)),(280,210,80,60),None


def forbidden(*args,**kwargs):
    raise AssertionError('UI冒烟测试不允许打开硬件')


if __name__=='__main__':
    import json
    cfg=json.loads((app.ROOT/'config.json').read_text())
    with patch.object(app,'Camera',FakeCamera), patch('devices.FeedbackChassis',forbidden), patch('devices.AirGrip',forbidden):
        win=tk.Tk()
        instance=app.App(win,cfg,app.ROOT,False)
        win.update()
        assert all(b['state']=='disabled' for b in instance.buttons)
        assert [b['text'] for b in instance.buttons]==['移动并抓取','释放','停止抽气']
        assert win.winfo_width() <= 800, win.winfo_width()
        assert win.winfo_height() <= 480, win.winfo_height()
        print('[UI_CHECK] 800x480 fits:',win.winfo_width(),win.winfo_height())
        win.after(300,instance.close)
        win.mainloop()
        print('[UI_CHECK] preview buttons disabled, clean close, no hardware')
