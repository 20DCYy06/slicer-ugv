#!/usr/bin/env python3
"""A接通吸口，D/空格断开，Q退出。默认只读检查，--run才允许控制。"""
import argparse
import json
import os
from pathlib import Path
import select
import signal
import struct
import subprocess
import sys
import termios
import time
import tty
import serial
if __package__:
    from .protocol import encode_frame,parse_frame,parse_kv
else:
    from protocol import encode_frame,parse_frame,parse_kv

HERE=Path(__file__).resolve().parent


class AirGrip:
    def __init__(self,cfg):
        self.cfg=cfg; self.seq=0; self.buffer=bytearray(); self.status={}
        self.must_release=False; self.ser=None
        path=cfg['device']
        stat=os.stat(path)
        node=Path('/sys/dev/char/%d:%d/device'%(os.major(stat.st_rdev),os.minor(stat.st_rdev))).resolve()
        usb=next((p for p in [node,*node.parents] if (p/'idVendor').exists()),None)
        if usb is None or tuple((usb/n).read_text().strip() for n in ['idVendor','idProduct','serial'])!=(cfg['vid'],cfg['pid'],cfg['serial_number']):
            raise RuntimeError('USB身份与AirGrip配置不符，不打开其它设备')
        owner=subprocess.run(['fuser',os.path.realpath(path)],capture_output=True,text=True)
        if owner.returncode==0 and owner.stdout.strip():raise RuntimeError('串口已被占用，先关闭AirGrip其它UI/节点，PID='+owner.stdout.strip())
        port=serial.Serial(port=None,baudrate=cfg['baud'],timeout=.05,write_timeout=.5,exclusive=True)
        # 与官方驱动一样声明CDC主机已连接；实板DTR低时不返回串口应答。
        # 不发送Ctrl-D/REPL/复位命令。RTS不参与设备控制。
        port.dtr=True;port.rts=False;port.port=path
        port.open();self.ser=port

    def send(self,cmd,payload=''):
        self.seq+=1;seq=str(self.seq)
        data=encode_frame(seq,cmd,payload).encode('utf-8')
        if self.ser.write(data)!=len(data):raise IOError('串口指令发送不完整')
        return seq

    def receive(self):
        data=self.ser.read(self.ser.in_waiting or 1)
        if not data:return []
        self.buffer.extend(data);frames=[]
        while b'\n' in self.buffer:
            raw,_,rest=self.buffer.partition(b'\n');self.buffer[:]=rest
            text=raw.decode('utf-8',errors='replace')
            ok,frame=parse_frame(text)
            if not ok:
                print('[RX_WARNING]',frame,repr(text),flush=True)
                continue
            frames.append(frame)
            if frame['cmd'] in ('RSP','EVT'):self.status.update(parse_kv(frame['payload']))
        return frames

    def request(self,cmd,payload=''):
        seq=self.send(cmd,payload);end=time.monotonic()+self.cfg['ack_timeout_s']
        while time.monotonic()<end:
            for frame in self.receive():
                if frame['cmd']=='RSP' and frame['seq']==seq:
                    result=parse_kv(frame['payload'])
                    if result.get('code')!='OK':raise RuntimeError(cmd+'设备拒绝：'+str(result))
                    return result
        raise TimeoutError(cmd+'未收到确认；不能据此判断吸口已通/断')

    def set_pump(self,on):
        # ON发送后即视作可能通电，哪怕ACK丢失，正常退出仍尝试OFF。
        if on:self.must_release=True
        result=self.request('PUMP_ON' if on else 'PUMP_OFF')
        if not on:self.must_release=False
        print('[ACK]',('吸取已确认（不等于已经吸住）' if on else '释放已确认'),json.dumps(result,ensure_ascii=False),flush=True)
        return result

    def close(self):
        if self.ser is None:return
        try:
            if self.must_release:
                try:self.set_pump(False)
                except Exception as exc:print('[WARNING] 退出释放未确认：%s；设备断线不会自动释放，请现场处理！'%exc,flush=True)
        finally:self.ser.close()


def run(args):
    cfg=json.loads((HERE/'config.json').read_text())
    if args.run and not sys.stdin.isatty():raise RuntimeError('请在交互式SSH终端运行')
    dev=AirGrip(cfg);old=None;fd=None
    def interrupted(*unused):raise KeyboardInterrupt()
    for sig in (signal.SIGINT,signal.SIGTERM,signal.SIGHUP):signal.signal(sig,interrupted)
    try:
        print('[PING]',json.dumps(dev.request('PING'),ensure_ascii=False),flush=True)
        print('[STATUS]',json.dumps(dev.request('GET_STATUS'),ensure_ascii=False),flush=True)
        if not args.run:return
        fd=sys.stdin.fileno();old=termios.tcgetattr(fd);tty.setcbreak(fd)
        print('A 接通吸取；D/空格 断开释放；Q/Ctrl+C退出。无需回车。启动不改变泵状态。',flush=True)
        print('断线不会自动释放；本程序操作过的吸取，在正常退出时会尝试释放。',flush=True)
        last_link=last_status=0.;previous=None
        while True:
            now=time.monotonic()
            if select.select([fd],[],[],0)[0]:
                raw=os.read(fd,64)
                if not raw:break
                # 一次读取中的长按重复只执行一次，不积压重复动作。
                last=None
                for key in raw.decode(errors='ignore').lower():
                    if key in ('q','\x03','\x1b'):return
                    if key in ('a','d',' '):last=key
                if last is not None:
                    try:dev.set_pump(last=='a')
                    except Exception as exc:print('[ERROR]',exc,flush=True)
            if now-last_link>=cfg['link_refresh_s']:
                dev.send('SET_LINK','connected=1');last_link=now
            if now-last_status>=cfg['status_poll_s']:
                dev.send('GET_STATUS');last_status=now
            dev.receive()
            state={k:dev.status[k] for k in ['system_state','pump','attach','valve','v','fw'] if k in dev.status}
            if state!=previous:
                print('[STATUS]',json.dumps(state,ensure_ascii=False),flush=True);previous=state
    finally:
        try:dev.close()
        finally:
            if old is not None:termios.tcsetattr(fd,termios.TCSANOW,old)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    g=p.add_mutually_exclusive_group();g.add_argument('--run',action='store_true');g.add_argument('--check',action='store_true')
    try:run(p.parse_args())
    except KeyboardInterrupt:print('\n已退出。')
    except Exception as exc:print('[ERROR]',exc,file=sys.stderr);sys.exit(1)
