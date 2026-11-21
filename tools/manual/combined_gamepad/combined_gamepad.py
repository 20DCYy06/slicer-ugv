#!/usr/bin/env python3
"""双摇杆底盘 + Y/A按住舵机 + X/B吸放；支持PB14及原PB15固件。"""
import argparse
from collections import deque
import json
import os
from pathlib import Path
import queue
import select
import signal
import struct
import sys
import termios
import threading
import time
import tty

HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE.parent))
from control_core import Chassis,Gear,Slew,Joystick,EVENT,gamepad_config,gamepad_target,load_config
from airgrip.keyboard import AirGrip, ping_on_open
from c30d_servo import CAPABILITY_QUERY, CAPABILITY_REPLY, PULSE_COMMAND, decode_capability, pulse_payload


def wheel_pwm_log(pwm_ticks,period):
    """0x90遥测的有符号PWM幅值占比；period取自0x85确认，不按80%上限归一化。"""
    values=' '.join('%s=%.1f%%'%(name,abs(value)*100.0/period)
                    for name,value in zip('ABCD',pwm_ticks))
    return '[PWM] %s ticks=%s'%(values,tuple(pwm_ticks))


def servo_target(held,mapping,cfg):
    cw=mapping['clockwise'] in held;ccw=mapping['counterclockwise'] in held
    if cw==ccw:return cfg['neutral_us']
    return cfg['clockwise_us'] if cw else cfg['counterclockwise_us']


def pump_target(pressed,mapping):
    if mapping['release'] in pressed:return False
    if mapping['suction'] in pressed:return True
    return None


class HeldJoystick(Joystick):
    """沿用Linux js事件与轴定义，增加松键状态；开机初始化事件不作为动作触发。"""
    def __init__(self,path):
        super().__init__(path);self.held=set()

    def poll(self):
        pressed=[]
        self.button_events=[]
        if not os.path.exists(self.path):raise RuntimeError('手柄断开')
        while True:
            try:data=os.read(self.fd,EVENT.size)
            except BlockingIOError:break
            if len(data)!=EVENT.size:raise RuntimeError('手柄读取中断')
            ms,value,kind,n=EVENT.unpack(data)
            if kind&2:self.axes[n]=max(-1.,min(1.,value/32767.))
            if kind&1 and not kind&0x80:
                if value:
                    if n not in self.held:
                        pressed.append(n);self.button_events.append((n,True))
                    self.held.add(n)
                else:
                    if n in self.held:self.button_events.append((n,False))
                    self.held.discard(n)
        return self.axes,pressed


class CombinedChassis(Chassis):
    """四轮0x16与舵机0x31共用一个连接；从能力回复识别实际输出引脚。"""
    def servo_check(self):
        self.replies.pop(CAPABILITY_REPLY, None)
        self.send(CAPABILITY_QUERY)
        self.wait(lambda: CAPABILITY_REPLY in self.replies)
        pin, _, _ = decode_capability(self.replies[CAPABILITY_REPLY])
        self.servo_pin=pin
        print('[SERVO] PB%d/TIM12_CH%d接口已确认'%(pin,1 if pin==14 else 2),flush=True)
        if pin==15:print('[NOTICE] 当前固件使用PB15，请核对舵机接线。',flush=True)

    def servo_pulse(self,value):
        self.send(PULSE_COMMAND, pulse_payload(value))


class PumpWorker:
    """独占AirGrip串口。ACK等待在此线程执行，不阻塞50Hz底盘循环。"""
    def __init__(self,cfg,device_factory=AirGrip):
        self.cfg=cfg;self.factory=device_factory;self.ready=threading.Event();self.stop_event=threading.Event()
        self.condition=threading.Condition();self.pending=None;self.error=None;self.messages=queue.Queue()
        self.thread=threading.Thread(target=self.run,name='airgrip-serial',daemon=True)

    def start(self):
        self.thread.start()
        if not self.ready.wait(5):raise RuntimeError('AirGrip初始化未完成')
        if self.error:raise RuntimeError('AirGrip初始化失败：'+str(self.error))

    def command(self,on):
        # 只保留最新按钮意图；正在等待前一个ACK时不积压旧吸取命令。
        with self.condition:self.pending=on;self.condition.notify()

    def run(self):
        dev=None
        try:
            dev=self.factory(self.cfg)
            self.messages.put(('READY',ping_on_open(dev)))
            self.messages.put(('STATUS',dev.request('GET_STATUS')))
            self.ready.set();last_link=last_status=0.;last_state=None
            while not self.stop_event.is_set():
                with self.condition:
                    target=self.pending;self.pending=None
                if target is not None:
                    try:
                        result=dev.set_pump(target);self.messages.put(('PUMP_ACK',result))
                    except TimeoutError as exc:self.messages.put(('PUMP_ERROR',str(exc)))
                    except RuntimeError as exc:self.messages.put(('PUMP_ERROR',str(exc)))
                if self.stop_event.is_set():break
                now=time.monotonic()
                if now-last_link>=self.cfg['link_refresh_s']:
                    dev.send('SET_LINK','connected=1');last_link=now
                if now-last_status>=self.cfg['status_poll_s']:
                    dev.send('GET_STATUS');last_status=now
                dev.receive()
                # 电压不断微动，不把每一帧ADC噪声打印到终端。
                state={k:dev.status[k] for k in ('system_state','pump','attach','fw') if k in dev.status}
                if state!=last_state:self.messages.put(('STATUS',state));last_state=state
                self.stop_event.wait(.01)
        except Exception as exc:
            self.error=exc;self.messages.put(('ERROR',str(exc)))
        finally:
            self.ready.set()
            if dev:dev.close()  # 只释放本程序可能启动过的吸取；固件失联不会自行释放。

    def close(self):
        self.stop_event.set()
        with self.condition:self.pending=None;self.condition.notify()
        if self.thread.ident is not None:
            self.thread.join(2)
            if self.thread.is_alive():print('[WARNING] 吸口线程未确认退出释放，请现场检查。',flush=True)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run',action='store_true',help='连接硬件，允许手柄控制')
    parser.add_argument('--config',default=str(HERE/'combined_gamepad.json'))
    args=parser.parse_args()
    if not args.run:
        parser.print_help();print('离线检查：./tools/manual/run_combined_gamepad.sh --test');return
    p=Path(args.config).resolve();combined=json.loads(p.read_text())
    def path(k):return (p.parent/combined[k]).resolve()
    cfg=gamepad_config(load_config(path('hardware_config')))
    servo=json.loads(path('servo_config').read_text());pumpcfg=json.loads(path('airgrip_config').read_text())
    mapping=combined['buttons']
    if len(set(mapping.values()))!=4:raise ValueError('四个功能按键编号不能重复')
    if servo['neutral_us']!=1500:raise ValueError('PB15中位应与固件1500us一致')
    for key in ['clockwise_us','counterclockwise_us']:
        if not 500<=servo[key]<=2500:raise ValueError('舵机PWM超出500..2500us')
    joy=car=pump=None;old=None;servo_used=False
    def interrupted(*unused):raise KeyboardInterrupt()
    for sig in (signal.SIGINT,signal.SIGTERM,signal.SIGHUP):signal.signal(sig,interrupted)
    try:
        joy=HeldJoystick(cfg['gamepad']['device'])
        pump=PumpWorker(pumpcfg);pump.start()
        car=CombinedChassis(cfg);car.connect();car.servo_check()
        gear=Gear(cfg);slew=Slew(cfg)
        if sys.stdin.isatty():old=termios.tcgetattr(sys.stdin.fileno());tty.setcbreak(sys.stdin.fileno())
        print('左Y油门 / 右X方向；Y按住顺转 / A按住逆转，松开停；X吸 / B放。',flush=True)
        print('RB/LB升降档；终端R/E调档、空格停底盘+舵机（不释放吸口）、Q退出。B不再停车。',flush=True)
        print(gear.text(),flush=True)
        button_names={mapping['clockwise']:'Y/舵机顺转',mapping['counterclockwise']:'A/舵机逆转',
                      mapping['suction']:'X/吸取',mapping['release']:'B/释放'}
        last=time.monotonic();last_servo=last_log=0.;previous_servo=1500;servo_cancel=False
        while True:
            now=time.monotonic();axes,pressed=joy.poll();car.healthy()
            for number,down in joy.button_events:
                print('[BUTTON] %s id=%d %s'%(button_names.get(number,'按钮'),number,'DOWN' if down else 'UP'),flush=True)
            if pump.error:raise RuntimeError('AirGrip链路失败：'+str(pump.error))
            while not pump.messages.empty():
                name,message=pump.messages.get_nowait();print('[AIRGRIP_'+name+']',message,flush=True)
            for button in pressed:
                if button==cfg['gamepad']['up_button']:print(gear.change(1),flush=True)
                elif button==cfg['gamepad']['down_button']:print(gear.change(-1),flush=True)
            target=pump_target(pressed,mapping)
            if target is not None:pump.command(target)
            stop=False
            if old is not None and select.select([sys.stdin],[],[],0)[0]:
                keys=os.read(sys.stdin.fileno(),64).decode(errors='ignore').lower()
                if any(k in keys for k in ('q','\x1b','\x03')):break
                for key in keys:
                    if key in ('r','e'):print(gear.change(1 if key=='r' else -1),flush=True)
                    elif key==' ':stop=True
            if stop:joy.axes={};axes={};servo_cancel=True
            if mapping['clockwise'] not in joy.held and mapping['counterclockwise'] not in joy.held:servo_cancel=False
            pulse=1500 if servo_cancel else servo_target(joy.held,mapping,servo)
            if pulse!=previous_servo or (pulse!=1500 and now-last_servo>=1/combined['servo_refresh_hz']):
                servo_used=True;car.servo_pulse(pulse);last_servo=now
                if pulse!=previous_servo:print('[PB%d]'%car.servo_pin,pulse,'us',flush=True)
                previous_servo=pulse
            v,w=gamepad_target(axes,gear,cfg['gamepad']);v,w=slew.step(v,w,now-last)
            car.speed(v,w);last=now
            if now-last_log>.5:
                print('[DRIVE] vx=%.3f wz=%.3f %s'%(v,w,wheel_pwm_log(car.state['pwm_ticks'],struct.unpack('<IBHHH',car.replies[0x85])[3])),flush=True);last_log=now
            time.sleep(1/cfg['control']['publish_hz'])
    finally:
        # 不等待泵ACK就先停底盘和舵机；任何一路清理出错都继续清理其它连接。
        if car:
            try:
                if servo_used:car.servo_pulse(1500)
            except Exception as exc:print('[STOP_SERVO_ERROR]',exc,flush=True)
            try:car.close()
            except Exception as exc:print('[STOP_DRIVE_ERROR]',exc,flush=True)
        if joy:joy.close()
        if pump:pump.close()
        if old is not None:termios.tcsetattr(sys.stdin.fileno(),termios.TCSANOW,old)
        print('[COMBINED] 已退出；未恢复旧指令。',flush=True)


if __name__=='__main__':
    try:main()
    except KeyboardInterrupt:pass
    except Exception as exc:print('[ERROR]',exc,file=sys.stderr);sys.exit(1)
