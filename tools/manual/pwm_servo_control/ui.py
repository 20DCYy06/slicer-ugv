"""8路按住转动、松开停止；9路滑块位置控制。启动不发送默认位置。"""
import argparse
import json
import os
from pathlib import Path
import queue
import signal
import subprocess
import sys
import threading
import tkinter as tk
from tkinter import ttk
from protocol import load_config, speed_pulse, angle_pulse
from hid_bridge import boards

HERE = Path(__file__).resolve().parent


class App:
    def __init__(self, root, demo=False):
        self.root, self.demo = root, demo
        self.cfg = load_config(HERE / 'config.json')
        self.events = queue.Queue()
        self.proc = None
        self.pending = {}
        self.sent = []
        self.ready = demo
        self.closing = False
        self.holding8 = False
        self.silent = False
        root.title('PWM舵机：8路连续旋转 / 9路位置')
        root.geometry('780x440')
        root.option_add('*Font', 'Sans 17')
        style = ttk.Style(root)
        style.configure('TButton', font=('Sans', 16), padding=6)
        style.configure('TLabel', font=('Sans', 16))
        frame = ttk.Frame(root, padding=12)
        frame.pack(fill='both', expand=True)
        self.status = tk.StringVar(value='演示模式：不连接硬件' if demo else '连接控制板中……（不会自动回中）')
        ttk.Label(frame, textvariable=self.status, wraplength=750).pack(anchor='w')
        ttk.Label(frame, text='8路：按住才转，松开即停').pack(anchor='w', pady=(10, 0))
        self.speed_text = tk.StringVar(value='已停止（启动尚未发送）')
        ttk.Label(frame, textvariable=self.speed_text).pack(anchor='w')
        turn_row = ttk.Frame(frame)
        turn_row.pack(fill='x', pady=4)
        self.cw = ttk.Button(turn_row, text='顺时针（按住）')
        self.ccw = ttk.Button(turn_row, text='逆时针（按住）')
        for button, direction in ((self.cw, 1), (self.ccw, -1)):
            button.pack(side='left', expand=True, fill='x', padx=4)
            button.bind('<ButtonPress-1>', lambda event, d=direction: self.press8(d))
            button.bind('<Leave>', lambda event: self.release8())
        ttk.Label(frame, text='9路：位置 / 标称角度（不是位置回读）').pack(anchor='w', pady=(8, 0))
        self.angle_text = tk.StringVar(value='中点预览（尚未发送，拖动才生效）')
        ttk.Label(frame, textvariable=self.angle_text).pack(anchor='w')
        p = self.cfg['channel9']
        self.angle = tk.Scale(frame, from_=p['angle_min_deg'], to=p['angle_max_deg'], resolution=1,
                              orient='horizontal', showvalue=False, width=24, sliderlength=34)
        self.angle.set((p['angle_min_deg'] + p['angle_max_deg']) / 2)
        self.angle.pack(fill='x')
        self.angle.configure(command=self.change9)
        row = ttk.Frame(frame)
        row.pack(fill='x', pady=10)
        ttk.Button(row, text='停止8路（空格）', command=self.stop8).pack(side='left')
        ttk.Button(row, text='读取板电压', command=lambda: self.send({'op': 'voltage'})).pack(side='left', padx=8)
        ttk.Button(row, text='关闭', command=self.close).pack(side='right')
        self.last = tk.StringVar(value='PWM仅下发目标；此板不提供舵机实际角度/磁编码器反馈。')
        ttk.Label(frame, textvariable=self.last, wraplength=750).pack(anchor='w')
        # 松开鼠标、移出按钮、切换到别的窗口或最小化，都结束按住动作。
        root.bind_all('<ButtonRelease-1>', lambda event: self.release8(), add='+')
        root.bind('<FocusOut>', lambda event: root.after_idle(self.check_focus8), add='+')
        root.bind('<Unmap>', lambda event: self.release8() if event.widget is root else None, add='+')
        root.bind('<space>', lambda event: self.stop8())
        root.bind('<Escape>', lambda event: self.stop8())
        root.protocol('WM_DELETE_WINDOW', self.close)
        # 等Tk完成初始滑块布局后再打开回调，避免初始化set触发MOVE。
        self.silent = True
        root.after_idle(self.start)
        root.after(50, self.poll)

    def start(self):
        self.pending.clear()
        self.silent = False
        if self.demo:
            return
        paths = boards()
        if len(paths) != 1:
            self.status.set('未找到唯一Hiwonder控制板：' + str(paths))
            return
        cmd = ['/usr/bin/python3', '-s', '-u', str(HERE / 'hid_bridge.py')]
        if not os.access(paths[0], os.R_OK | os.W_OK):
            cmd = ['sudo', '-n'] + cmd
        try:
            self.proc = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                         stderr=subprocess.STDOUT, text=True, bufsize=1)
            threading.Thread(target=self.read_events, daemon=True).start()
        except OSError as exc:
            self.status.set(str(exc))

    def read_events(self):
        for line in self.proc.stdout:
            try:
                self.events.put(json.loads(line))
            except ValueError:
                self.events.put({'event': 'error', 'message': line.strip()})
        self.events.put({'event': 'disconnected'})

    def press8(self, direction):
        if not self.ready or self.closing:
            self.last.set('尚未连接控制板；连接后请重新按住按钮。')
            return 'break'
        v = direction * self.cfg['channel8'].get('hold_speed_percent', 100)
        self.holding8 = True
        button = self.cw if direction > 0 else self.ccw
        button.focus_set()
        button.state(['pressed'])
        self.speed_text.set('%s / %.0f%% / %d μs' % (
            '顺时针' if direction > 0 else '逆时针', abs(v), speed_pulse(v, self.cfg['channel8'])))
        # 按钮指令立即发送，不进入9路滑块的合并队列，快速点按也能立即停止。
        self.send({'op': 'speed8', 'value': v})
        return 'break'

    def release8(self):
        if self.holding8:
            self.stop8()

    def check_focus8(self):
        if self.closing or not self.holding8:
            return
        focus = self.root.focus_displayof()
        if focus is None or focus.winfo_toplevel() is not self.root:
            self.release8()

    def change9(self, value):
        v = float(value)
        p = angle_pulse(v, self.cfg['channel9'])
        self.angle_text.set('%.0f°（标称） / %d μs' % (v, p))
        if not self.silent and self.ready:
            self.pending[9] = {'op': 'angle9', 'value': v}

    def send(self, req):
        if not self.ready:
            self.last.set('尚未连接控制板；连接后请重新操作。')
            return
        if self.demo:
            self.sent.append(req)
            self.last.set('演示，未发送：' + json.dumps(req, ensure_ascii=False))
            return
        try:
            self.proc.stdin.write(json.dumps(req) + '\n')
            self.proc.stdin.flush()
        except (OSError, ValueError) as exc:
            self.ready = False
            self.status.set('连接断开：' + str(exc))

    def stop8(self):
        self.holding8 = False
        self.cw.state(['!pressed'])
        self.ccw.state(['!pressed'])
        self.speed_text.set('已停止 / %d μs' % self.cfg['channel8']['neutral_us'])
        self.pending.pop(8, None)
        self.send({'op': 'stop8'})

    def poll(self):
        if self.closing:
            return
        while not self.events.empty():
            e = self.events.get_nowait()
            event = e['event']
            print('[PWM]', json.dumps(e, ensure_ascii=False), flush=True)
            if event == 'connected':
                self.ready = True
                self.pending.clear()
                mv = e['voltage_mv']
                self.status.set('已连接 %s / %s；按住8路按钮或拖动9路才发送' % (e['device'], ('%.2f V' % (mv / 1000)) if mv is not None else '电压未回读'))
            elif event == 'voltage':
                self.last.set('控制板电压：' + str(e['voltage_mv']) + ' mV')
            elif event == 'sent':
                self.last.set('已发送：%d路 %d μs（不是到位确认）' % (e['channel'], e['pulse_us']))
            elif event == 'error':
                self.last.set('错误：' + e['message'])
            elif event == 'disconnected':
                self.ready = False
                self.holding8 = False
                self.cw.state(['!pressed'])
                self.ccw.state(['!pressed'])
                self.pending.clear()
                self.status.set('控制板连接已断开；检查USB/权限后重新启动')
        # 20Hz合并滑块事件，只发送每一路最新值，不积压过时运动命令。
        pending, self.pending = self.pending, {}
        for req in pending.values():
            self.send(req)
        self.root.after(50, self.poll)

    def close(self):
        if self.closing:
            return
        self.closing = True
        self.pending.clear()
        if self.proc and self.proc.poll() is None:
            try:
                self.proc.stdin.write('{"op":"quit"}\n')
                self.proc.stdin.flush()
                self.proc.stdin.close()
                self.proc.wait(timeout=2)
            except (OSError, ValueError, subprocess.TimeoutExpired):
                print('[PWM] 退出时未确认8路停止，请检查电源/USB；必要时切断舵机供电。', flush=True)
        self.root.destroy()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--demo', action='store_true')
    parser.add_argument('--self-test', action='store_true', help='无硬件的真实Tk窗口检查')
    args = parser.parse_args()
    root = tk.Tk()
    app = App(root, args.demo or args.self_test)
    signal.signal(signal.SIGTERM, lambda *a: root.after(0, app.close))
    signal.signal(signal.SIGINT, lambda *a: root.after(0, app.close))
    if args.self_test:
        def exercise():
            assert not app.sent, '初始化不得发送MOVE'
            app.press8(1)
            assert app.sent[-1] == {'op': 'speed8', 'value': app.cfg['channel8'].get('hold_speed_percent', 100)}
            app.cw.event_generate('<ButtonRelease-1>')
            root.update()
            assert app.sent[-1]['op'] == 'stop8' and not app.holding8
            app.press8(-1)
            assert app.sent[-1]['value'] < 0
            app.ccw.event_generate('<Leave>')
            root.update()
            assert app.sent[-1]['op'] == 'stop8' and not app.holding8
            app.change9(100)
            app.poll()
            assert app.sent[-1]['op'] == 'angle9'
            print('GUI_SELF_TEST_PASS: 按住双向、松开/移出停止、9路滑块、启动无动作；无硬件', flush=True)
            app.close()
        root.after(300, exercise)
    root.mainloop()


if __name__ == '__main__':
    main()
