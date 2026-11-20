#!/usr/bin/env python3
"""PB14/PB15连续舵机：A/D点动，实际输出引脚由固件能力查询确认。"""
import argparse
import json
import os
from pathlib import Path
import secrets
import select
import signal
import struct
import sys
import termios
import time
import tty

HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(HERE.parents[2]/'Hardware/Host'))
from protocol import packet,Parser
from transport import Link
from chassis import device,load_config


def key_pulse(key,cfg):
    return cfg['clockwise_us'] if key.lower()=='a' else cfg['counterclockwise_us'] if key.lower()=='d' else cfg['neutral_us']


class Servo:
    def __init__(self):
        path,expected=device(load_config())
        self.link=Link(path,expected,restore_on_close=False)
        self.parser=Parser(); self.seq=secrets.randbits(16)
        self.path=path; self.moved=False

    def send(self,kind,data=b''):
        self.seq=(self.seq+1)&65535
        self.link.send(packet(kind,self.seq,data))

    def receive(self,wait=0):
        if not select.select([self.link.fd],[],[],wait)[0]:return []
        raw=os.read(self.link.fd,4096)
        if not raw:raise RuntimeError('底盘USB已断开')
        return self.parser.feed(raw)

    def check(self):
        # 不用Chassis.connect：HELLO会重置车轮会话；这里只查询PB15能力。
        self.send(0x30)
        end=time.monotonic()+1.5
        while time.monotonic()<end:
            for kind,seq,data in self.receive(.05):
                if kind==0x86 and len(data)==10:
                    magic,version,pin,neutral,watchdog=struct.unpack('<4sBBHH',data)
                    if (magic,pin) not in ((b'B14P',14),(b'B15P',15)) or version!=1 or neutral!=1500:
                        raise RuntimeError('舵机能力接口不匹配')
                    self.pin=pin
                    print('[READY]',self.path,'PB%d/TIM12_CH%d 50Hz neutral=%dus watchdog=%dms'%(pin,1 if pin==14 else 2,neutral,watchdog),flush=True)
                    if pin==15:print('[NOTICE] 当前固件仍输出PB15；刷入v1.4后才会切换到PB14。',flush=True)
                    return
        raise RuntimeError('未收到舵机能力回复；PB14候选固件在Hardware/Firmware/v1.4/bin/firmware.bin，本次未发送运动指令。')

    def pulse(self,value):
        self.moved=True
        self.send(0x31,struct.pack('<H',value))

    def close(self):
        try:
            if self.moved:self.send(0x31,struct.pack('<H',1500))
        finally:self.link.close()


def main():
    p=argparse.ArgumentParser(description=__doc__)
    mode=p.add_mutually_exclusive_group()
    mode.add_argument('--run',action='store_true')
    mode.add_argument('--check',action='store_true')
    mode.add_argument('--self-test',action='store_true')
    p.add_argument('--clockwise-us',type=int,help='临时覆盖A键脉宽，不修改配置文件')
    p.add_argument('--counterclockwise-us',type=int,help='临时覆盖D键脉宽，不修改配置文件')
    args=p.parse_args()
    cfg=json.loads((HERE/'config.json').read_text())
    if args.clockwise_us is not None:cfg['clockwise_us']=args.clockwise_us
    if args.counterclockwise_us is not None:cfg['counterclockwise_us']=args.counterclockwise_us
    assert cfg['neutral_us']==1500,'中位应与候选固件1500us一致'
    for name in ['clockwise_us','counterclockwise_us']:assert 500<=cfg[name]<=2500
    assert 0<cfg['tap_duration_s']<=1 and 5<=cfg['refresh_hz']<=100
    if args.self_test:
        assert [key_pulse(k,cfg) for k in ['a','d',' ']]==[cfg['clockwise_us'],cfg['counterclockwise_us'],1500]
        frame=packet(0x31,3,struct.pack('<H',2000))
        assert Parser().feed(frame)==[(0x31,3,struct.pack('<H',2000))]
        print('OFFLINE_PASS：A/D映射、停止、SLC2帧；未打开串口')
        return
    if args.run and not sys.stdin.isatty():raise RuntimeError('请在交互式SSH终端运行')
    servo=Servo();old=None;fd=None
    def interrupted(*a):raise KeyboardInterrupt()
    for sig in [signal.SIGINT,signal.SIGTERM,signal.SIGHUP]:signal.signal(sig,interrupted)
    try:
        servo.check()
        if not args.run:return
        fd=sys.stdin.fileno();old=termios.tcgetattr(fd);tty.setcbreak(fd)
        print('A 顺时针 / D 逆时针点动；长按重复继续；空格停；Q/Ctrl+C退出。无需回车。',flush=True)
        print('SSH没有松键事件：每次按键点动%.2fs，随后发1500us停止。方向若反了交换config中的两个端点。'%cfg['tap_duration_s'],flush=True)
        print('[CONFIG] A=%dus D=%dus STOP=%dus；ACK仅证明下位机接收，不代表舵机运动。'%(cfg['clockwise_us'],cfg['counterclockwise_us'],cfg['neutral_us']),flush=True)
        value=1500;until=0.;previous=None
        previous_ack=None
        while True:
            now=time.monotonic()
            if select.select([fd],[],[],0)[0]:
                raw=os.read(fd,64)
                if not raw:break
                for key in raw.decode(errors='ignore').lower():
                    if key in ('q','\x03','\x1b'):return
                    if key in ('a','d',' '):
                        value=key_pulse(key,cfg);until=now+cfg['tap_duration_s'] if key!=' ' else now
            if now>=until:value=1500
            if value!=1500 or previous!=value:
                servo.pulse(value)
                if previous!=value:print('[PB%d] %dus'%(servo.pin,value),flush=True)
                previous=value
            # 只展示脉宽切换后的回显，避免20Hz刷屏；不把回显当作位置反馈。
            for kind,seq,data in servo.receive(0):
                if kind==0x87 and len(data)==2:
                    ack=struct.unpack('<H',data)[0]
                    if ack!=previous_ack:print('[PB%d_ACK] 下位机已接受 %dus'%(servo.pin,ack),flush=True)
                    previous_ack=ack
            time.sleep(1/cfg['refresh_hz'])
    finally:
        try:servo.close()
        finally:
            if old is not None:termios.tcsetattr(fd,termios.TCSANOW,old)


if __name__=='__main__':
    try:main()
    except KeyboardInterrupt:print('\n已停止。')
    except Exception as exc:print('[ERROR]',exc,file=sys.stderr);sys.exit(1)
