#!/usr/bin/env python3
"""尾部相机视觉对准+吸口抓取。默认只预览，--run才启用动作按钮。"""
import argparse
import json
import faulthandler
import fcntl
import logging
import os
from pathlib import Path
import queue
import signal
import sys
import threading
import time
import tkinter as tk
from tkinter import messagebox, ttk
import cv2
from PIL import Image, ImageTk
from .vision import Camera
from .pipeline import Runtime, save_json
from .paths import DEFAULT_CONFIG, DEVICE_CONFIGS, device_config


class App:
    def __init__(self,win,cfg,root,enabled,config_path=None):
        self.win,self.cfg,self.root,self.enabled=win,cfg,root,enabled
        self.config_path = config_path if config_path is not None else root / "config.json"
        self.messages=queue.Queue();self.cancel=threading.Event()
        self.busy=self.closing=self.closed=self.reconnecting=False
        self.stopping_pump=False
        self.action_done=threading.Event();self.action_done.set()
        self.signal_exit_requested=False;self.close_started=None;self.refresh_job=None
        self.last_camera_error=None;self.drag=self.selection=None
        self.camera=Camera(cfg['camera'],cfg['target'],root);self.camera.start()
        win.title('Visual Grasp — 单次位移抓取'+(' / 可执行' if enabled else ' / 只预览'))
        win.option_add('*Font','Sans 14')
        bar=tk.Frame(win);bar.pack(fill='x');self.buttons=[]
        for label,action in [('移动并抓取',self.pick),('释放',self.release),('停止抽气',self.stop_pump)]:
            button=tk.Button(bar,text=label,command=action,height=2)
            button.pack(side='left',fill='x',expand=True);self.buttons.append(button)
        self.pump_stop_button=self.buttons[-1]
        self.hint=tk.Label(win,text='静止后锁定一次XY；Esc/空格停止，右键微调原点，R重连相机',font=('Sans',11),wraplength=750)
        self.hint.pack(fill='x')
        self.scale=min(1.,max(300,win.winfo_screenwidth()-290)/cfg['camera']['width'],max(220,win.winfo_screenheight()-140)/cfg['camera']['height'])
        body=tk.Frame(win);body.pack(fill='both',expand=True)
        self.canvas=tk.Canvas(body,width=round(cfg['camera']['width']*self.scale),height=round(cfg['camera']['height']*self.scale),bg='#20252a')
        self.canvas.pack(side='left')
        self.logs=tk.Text(body,width=29,height=12,state='disabled',font=('Monospace',11));self.logs.pack(side='left',fill='both',expand=True)
        self.canvas.bind('<ButtonPress-1>',self.start_box);self.canvas.bind('<B1-Motion>',self.move_box)
        self.canvas.bind('<ButtonRelease-1>',self.end_box);self.canvas.bind('<Button-3>',self.set_origin)
        tk.Label(win,text='Esc/空格停底盘；停止抽气会关泵并可能释放物块。本程序不控制升降。',font=('Sans',11),fg='#a02020').pack()
        win.bind('<Escape>',lambda e:self.stop());win.bind('<space>',lambda e:self.stop())
        win.bind('<r>',lambda e:self.reconnect_camera());win.bind('<R>',lambda e:self.reconnect_camera())
        win.protocol('WM_DELETE_WINDOW',self.close)
        self.log('[READY] 移动并抓取 / 释放 / 停止抽气；一次识别加3cm靠近补偿，不控制升降舵机。')
        self.refresh()

    def log(self,text):
        logging.getLogger('visual_grasp').info(text)
        try:print(text,flush=True)
        except (OSError,ValueError):pass
        self.messages.put(('log',text))

    def start_box(self,e):
        if not self.busy:self.drag=(round(e.x/self.scale),round(e.y/self.scale))

    def move_box(self,e):
        if self.drag:self.selection=(*self.drag,round(e.x/self.scale),round(e.y/self.scale))

    def end_box(self,e):
        if self.drag is None:return
        x,y=self.drag;self.drag=self.selection=None
        u=max(0,min(self.cfg['camera']['width']-1,round(e.x/self.scale)))
        v=max(0,min(self.cfg['camera']['height']-1,round(e.y/self.scale)))
        x,u=sorted((x,u));y,v=sorted((y,v))
        if u-x>=12 and v-y>=12:self.camera.select((x,y,u-x,v-y))

    def set_origin(self,e):
        if self.busy:return
        point=[max(0,min(self.cfg['camera']['width']-1,round(e.x/self.scale))),max(0,min(self.cfg['camera']['height']-1,round(e.y/self.scale)))]
        self.cfg['target']['nozzle']['origin_px']=point
        save_json(self.config_path,self.cfg)
        self.log('[ORIGIN] 红点像素=%s，定义为相对(0,0)'%point)

    def launch(self,operation):
        if self.busy or self.stopping_pump or self.closing or self.reconnecting or not self.enabled:return
        self.busy=True;self.cancel.clear();self.action_done.clear()
        def work():
            try:operation(Runtime(self.cfg,self.root,self.camera,self.cancel,self.log))
            except InterruptedError:self.log('[STOP] 已中断，底盘停止，泵保持当前状态')
            except Exception as exc:
                logging.getLogger('visual_grasp').exception('任务失败')
                self.log('[ERROR] '+str(exc))
            finally:self.action_done.set();self.messages.put(('finished',None))
        threading.Thread(target=work,daemon=True,name='snapshot-grasp').start()

    def pick(self):self.launch(lambda r:r.pick())
    def release(self):self.launch(lambda r:r.release())

    def stop_pump(self):
        """先中断旧任务并等它释放串口，再关泵，避免两个线程抢同一设备。"""
        if not self.enabled or self.closing or self.stopping_pump:return
        self.stopping_pump=True;self.cancel.set()
        self.log('[STOP_PUMP] 正在中断移动并关闭抽气')
        def work():
            try:
                self.action_done.wait()
                Runtime(self.cfg,self.root,self.camera,threading.Event(),self.log).release()
                self.log('[STOP_PUMP] 气泵关闭指令已确认')
            except Exception as exc:self.log('[STOP_PUMP_ERROR] '+str(exc))
            finally:self.messages.put(('pump_stopped',None))
        threading.Thread(target=work,daemon=True,name='stop-pump').start()

    def stop(self):
        self.cancel.set();self.log('[STOP] 已请求停止，不自动释放')

    def reconnect_camera(self):
        if self.busy or self.reconnecting or self.closing:return
        self.reconnecting=True;old=self.camera
        def reconnect():
            try:
                old.close()
                if old.thread.is_alive():raise RuntimeError('旧相机未释放，请重新插接USB')
                camera=Camera(self.cfg['camera'],self.cfg['target'],self.root);camera.start()
                self.messages.put(('camera_ready',camera))
            except Exception as exc:self.messages.put(('camera_failed',str(exc)))
        threading.Thread(target=reconnect,daemon=True).start()

    def refresh(self):
        if self.closed:return
        if self.refresh_job is not None:
            try:self.win.after_cancel(self.refresh_job)
            except tk.TclError:pass
            self.refresh_job=None
        try:
            if self.signal_exit_requested and not self.closing:self.close()
            if self.closing:
                if not self.busy and not self.reconnecting and not self.stopping_pump:
                    self.closed=True;self.win.destroy();return
                if time.monotonic()-self.close_started>=3:os._exit(130)
            self.refresh_view()
        except Exception as exc:
            self.cancel.set();logging.getLogger('visual_grasp').exception('界面刷新错误')
            self.log('[UI_ERROR] '+str(exc))
        finally:
            if not self.closed:self.refresh_job=self.win.after(100,self.refresh)

    def refresh_view(self):
        while not self.messages.empty():
            kind,value=self.messages.get_nowait()
            if kind=='finished':self.busy=False
            elif kind=='pump_stopped':self.stopping_pump=False
            elif kind=='camera_ready':
                self.camera=value;self.reconnecting=False;self.last_camera_error=None
                if self.closing:self.camera.close(wait=False)
                self.log('[CAMERA] 已重连')
            elif kind=='camera_failed':self.reconnecting=False;self.log('[CAMERA_ERROR] '+value)
            elif kind=='log':
                self.logs.config(state='normal');self.logs.insert('end',value+'\n');self.logs.see('end');self.logs.config(state='disabled')
        for b in self.buttons:
            available=self.enabled and not self.closing and not self.stopping_pump
            if b is not self.pump_stop_button:available=available and not self.busy and not self.reconnecting
            b.config(state='normal' if available else 'disabled')
        frame,obs,box,error=self.camera.snapshot()
        if error and error!=self.last_camera_error:
            self.last_camera_error=error;self.log('[CAMERA_ERROR] '+error+'；R重连')
        elif not error and self.last_camera_error:self.last_camera_error=None;self.log('[CAMERA] 预览已恢复')
        if frame is not None:
            if box and obs.center is not None:
                x,y,w,h=box;cv2.rectangle(frame,(x,y),(x+w,y+h),(0,255,0),2)
                cv2.drawMarker(frame,tuple(round(n) for n in obs.center),(0,255,0),0,18,2)
            if obs.nozzle_center is not None:
                point=tuple(round(n) for n in obs.nozzle_center)
                cv2.drawMarker(frame,point,(0,0,255),0,20,2)
                cv2.putText(frame,'(0,0)',(point[0]+12,point[1]-10),0,.5,(0,0,255),1)
                if obs.center is not None:cv2.arrowedLine(frame,point,tuple(round(n) for n in obs.center),(0,255,255),2)
            frame=cv2.resize(frame,None,fx=self.scale,fy=self.scale)
            self.photo=ImageTk.PhotoImage(Image.fromarray(cv2.cvtColor(frame,cv2.COLOR_BGR2RGB)))
            self.canvas.delete('all');self.canvas.create_image(0,0,anchor='nw',image=self.photo)
            if self.selection:self.canvas.create_rectangle(*(n*self.scale for n in self.selection),outline='yellow',width=2)
        if error:self.hint.config(text='相机错误：'+error)
        elif self.busy:self.hint.config(text='执行已锁定目标，不再重算视觉偏移；Esc/空格停止')
        elif obs.center is not None and obs.nozzle_center is not None:
            d=[obs.center[i]-obs.nozzle_center[i] for i in range(2)]
            self.hint.config(text='相对原点=(%.1f,%.1f)px；可见面7×10cm；正向对应画面朝下'%tuple(d))
        else:self.hint.config(text='等待粉色方块；右键微调原点，R重连相机，Esc停止')

    def close(self):
        if self.closing:return
        self.closing=True;self.close_started=time.monotonic();self.stop()
        self.camera.close(wait=False);self.win.withdraw()




def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', action='store_true', help='启用动作按钮；程序启动时仍不动作')
    parser.add_argument('--check', action='store_true', help='仅检查依赖和配置，不打开相机/串口')
    parser.add_argument('--config', type=Path)
    args = parser.parse_args()
    config_home = Path(os.environ.get('XDG_CONFIG_HOME') or Path.home() / '.config')
    path = args.config.resolve() if args.config else config_home / 'slicer_visual_grasp/config.json'
    read_path = DEFAULT_CONFIG if args.config is None and not path.is_file() else path
    cfg = json.loads(read_path.read_text())
    if args.check:
        for key in DEVICE_CONFIGS:
            dependency = device_config(cfg, path.parent, key)
            if not dependency.is_file():
                raise FileNotFoundError(dependency)
        print('[CHECK] OpenCV、NumPy、Tk、Pillow、串口依赖与路径检查通过；无硬件指令')
        return
    folder=path.parent/'data'
    folder.mkdir(parents=True, exist_ok=True)
    instance_lock=open(folder/'ui.lock','a+')
    try:
        fcntl.flock(instance_lock.fileno(),fcntl.LOCK_EX|fcntl.LOCK_NB)
    except BlockingIOError:
        raise RuntimeError('已有Visual Grasp窗口运行，请使用原窗口或先关闭它，避免重复占用相机')
    instance_lock.seek(0)
    instance_lock.truncate()
    instance_lock.write(str(os.getpid())+'\n')
    instance_lock.flush()
    handler=logging.FileHandler(folder/'runtime.log',encoding='utf-8')
    handler.setFormatter(logging.Formatter('%(asctime)s %(levelname)s %(message)s'))
    logger=logging.getLogger('visual_grasp')
    logger.setLevel(logging.INFO)
    logger.addHandler(handler)
    fault_file=open(folder/'thread_stacks.log','a')
    faulthandler.enable(file=fault_file,all_threads=True)
    faulthandler.register(signal.SIGUSR1,file=fault_file,all_threads=True)
    win = tk.Tk()
    app = App(win, cfg, path.parent, args.run, config_path=path)
    def exit_signal(signum, frame):
        # 信号回调中不调用Tk、不打印、不获取锁，防止重入造成二次死锁。
        if app.signal_exit_requested:
            os._exit(128+signum)
        app.signal_exit_requested = True
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, exit_signal)
    win.mainloop()


if __name__ == '__main__':
    try:
        main()
    except Exception as exc:
        print('[ERROR]', exc, file=sys.stderr)
        sys.exit(1)
