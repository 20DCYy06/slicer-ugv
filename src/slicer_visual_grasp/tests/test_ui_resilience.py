"""在Xvfb中执行：模拟相机掉线及退出，不连接任何真实设备。"""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import threading
import time
import tkinter as tk
import unittest
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np
from slicer_visual_grasp.paths import DEFAULT_CONFIG
from slicer_visual_grasp import app
from slicer_visual_grasp.logic import Observation

ROOT=Path(__file__).resolve().parent
CFG=json.loads(DEFAULT_CONFIG.read_text())


class BrokenCamera:
    def __init__(self,*args):
        self.calls=0;self.close_wait=[]
        self.thread=SimpleNamespace(is_alive=lambda:False)
    def start(self):pass
    def snapshot(self):
        self.calls+=1
        return np.zeros((480,640,3),np.uint8),Observation(self.calls,time.monotonic(),None),(5,5,30,30),'simulated disconnect'
    def close(self,wait=True):self.close_wait.append(wait)


class ResilienceTests(unittest.TestCase):
    def pump(self,win,duration):
        end=time.monotonic()+duration
        while time.monotonic()<end:
            try:win.update()
            except tk.TclError:break
            time.sleep(.01)

    def test_camera_loss_keeps_timer_stop_and_window_close_responsive(self):
        with tempfile.TemporaryDirectory() as folder,patch.object(app,'Camera',BrokenCamera):
            win=tk.Tk();a=app.App(win,CFG,Path(folder),False)
            self.pump(win,.25)
            self.assertGreaterEqual(a.camera.calls,3)
            self.assertIn('相机错误',a.hint['text'])
            a.stop();self.assertTrue(a.cancel.is_set())
            before=time.monotonic();a.close()
            self.assertLess(time.monotonic()-before,.1)
            self.assertEqual(a.camera.close_wait,[False])
            self.pump(win,.2)
            self.assertTrue(a.closed)

    def test_window_close_waits_for_camera_capture_to_release(self):
        released = threading.Event()

        class ClosingCamera(BrokenCamera):
            def __init__(self, *args):
                super().__init__(*args)
                self.thread = SimpleNamespace(is_alive=lambda: not released.is_set())

        with tempfile.TemporaryDirectory() as folder, patch.object(app, 'Camera', ClosingCamera):
            win = tk.Tk()
            try:
                a = app.App(win, CFG, Path(folder), False)
                a.close()
                self.pump(win, .25)
                self.assertFalse(a.closed)
                self.assertEqual(a.camera.close_wait, [False])
                released.set()
                self.pump(win, .25)
                self.assertTrue(a.closed)
            finally:
                try:
                    win.destroy()
                except tk.TclError:
                    pass

    def test_unexpected_render_error_does_not_remove_refresh_timer(self):
        class FailingOnce(BrokenCamera):
            def snapshot(self):
                self.calls+=1
                if self.calls==1:raise RuntimeError('injected render failure')
                return None,Observation(self.calls,time.monotonic(),None),None,None
        with tempfile.TemporaryDirectory() as folder,patch.object(app,'Camera',FailingOnce):
            win=tk.Tk();a=app.App(win,CFG,Path(folder),False)
            self.pump(win,.25)
            self.assertGreaterEqual(a.camera.calls,3)
            self.assertIsNotNone(a.refresh_job)
            a.close();self.pump(win,.2);self.assertTrue(a.closed)

    def test_reconnect_replaces_failed_camera_without_blocking_ui(self):
        with tempfile.TemporaryDirectory() as folder,patch.object(app,'Camera',BrokenCamera):
            win=tk.Tk();a=app.App(win,CFG,Path(folder),False);old=a.camera
            before=time.monotonic();a.reconnect_camera()
            self.assertLess(time.monotonic()-before,.1)
            self.pump(win,.3)
            self.assertFalse(a.reconnecting)
            self.assertIsNot(a.camera,old)
            self.assertEqual(old.close_wait,[True])
            a.close();self.pump(win,.2);self.assertTrue(a.closed)

    def test_stop_pump_is_available_during_motion_and_waits_for_serial_cleanup(self):
        order=[];started=threading.Event()
        class FakeRuntime:
            def __init__(self,cfg,root,camera,cancel,log):self.cancel=cancel
            def pick(self):
                started.set();self.cancel.wait()
                order.append('motion_and_serial_closed')
                raise InterruptedError('cancelled')
            def release(self):
                if order!=['motion_and_serial_closed']:raise AssertionError('串口尚未释放就试图关泵')
                order.append('pump_off')
        with tempfile.TemporaryDirectory() as folder,patch.object(app,'Camera',BrokenCamera),patch.object(app,'Runtime',FakeRuntime):
            win=tk.Tk();a=app.App(win,CFG,Path(folder),True)
            a.pick();self.assertTrue(started.wait(.5));a.refresh_view()
            self.assertEqual(a.buttons[0]['state'],'disabled')
            self.assertEqual(a.pump_stop_button['state'],'normal')
            before=time.monotonic();a.stop_pump();self.assertLess(time.monotonic()-before,.1)
            self.pump(win,.3)
            self.assertEqual(order,['motion_and_serial_closed','pump_off'])
            self.assertFalse(a.busy);self.assertFalse(a.stopping_pump)
            a.close();self.pump(win,.2)

    def test_stop_pump_also_works_when_idle(self):
        calls=[]
        class FakeRuntime:
            def __init__(self,*a):pass
            def release(self):calls.append('pump_off')
        with tempfile.TemporaryDirectory() as folder,patch.object(app,'Camera',BrokenCamera),patch.object(app,'Runtime',FakeRuntime):
            win=tk.Tk();a=app.App(win,CFG,Path(folder),True)
            a.stop_pump();self.pump(win,.2)
            self.assertEqual(calls,['pump_off']);self.assertFalse(a.stopping_pump)
            a.close();self.pump(win,.2)

    def probe(self,mode):
        temp=tempfile.TemporaryDirectory()
        directory=Path(temp.name);config=directory/'config.json';config.write_text(json.dumps(CFG))
        ready=directory/'ready';env=dict(os.environ,VG_PROBE_MODE=mode,VG_PROBE_READY=str(ready))
        process=subprocess.Popen([sys.executable,'-s',str(ROOT/'ui_exit_probe.py'),'--config',str(config)],
                                 env=env,stdout=subprocess.DEVNULL,stderr=subprocess.PIPE)
        self.addCleanup(temp.cleanup)
        def cleanup():
            if process.poll() is None:process.kill()
            process.communicate()
        self.addCleanup(cleanup)
        end=time.monotonic()+4
        while not ready.exists() and process.poll() is None and time.monotonic()<end:time.sleep(.02)
        self.assertTrue(ready.exists(),'fake UI failed to start')
        return process

    def test_ctrl_c_exits_after_camera_failure(self):
        p=self.probe('signal');p.send_signal(signal.SIGINT)
        self.assertEqual(p.wait(timeout=2),0)

    def test_window_close_exits_after_camera_failure(self):
        p=self.probe('x')
        self.assertEqual(p.wait(timeout=2),0)

    def test_duplicate_window_is_rejected_before_opening_camera(self):
        p=self.probe('signal')
        result=subprocess.run(p.args,env=dict(os.environ,VG_PROBE_MODE='signal',VG_PROBE_READY='/unused'),
                              capture_output=True,text=True,timeout=3)
        self.assertNotEqual(result.returncode,0)
        self.assertIn('已有Visual Grasp窗口运行',result.stderr)
        p.send_signal(signal.SIGINT);self.assertEqual(p.wait(timeout=2),0)

    def test_second_ctrl_c_forces_exit_when_cleanup_is_stuck(self):
        p=self.probe('busy');p.send_signal(signal.SIGINT);time.sleep(.2);p.send_signal(signal.SIGINT)
        self.assertEqual(p.wait(timeout=2),130)

    def test_window_close_has_bounded_cleanup_when_worker_is_stuck(self):
        p=self.probe('busy_x')
        self.assertEqual(p.wait(timeout=5),130)


if __name__=='__main__':unittest.main()
