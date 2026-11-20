"""仅使用0x16底盘和AirGrip；本程序不查询或控制升降舵机。"""
import os
import select
import struct
import sys
import threading
import time

from .paths import HARDWARE, MANUAL, device_config

sys.path[:0] = [str(HARDWARE / 'Host'), str(MANUAL)]
from airgrip.keyboard import AirGrip
from airgrip.protocol import parse_kv
from .logic import Pressure
from chassis import Chassis,Slew,load_config,status,body_status
from .snapshot_move import FeedbackPose


class FeedbackChassis(Chassis):
    """本程序专用反馈解码，不修改共享底盘驱动。"""
    def __init__(self,cfg):
        self.pose=FeedbackPose();self.last_body_rx=0.
        super().__init__(cfg)

    def poll(self,timeout=0.):
        if not select.select([self.link.fd],[],[],timeout)[0]:return
        raw=os.read(self.link.fd,4096)
        if not raw:raise RuntimeError('底盘USB已断开')
        for kind,seq,data in self.parser.feed(raw):
            if kind==0x90 and len(data)==61:
                self.state=status(data);self.last_rx=time.monotonic()
            elif kind==0x93 and len(data)==64:
                self.body=body_status(data)
                self.pose.update(struct.unpack_from('<I',data,4)[0],self.body['vx'],self.body['wz'])
                self.last_body_rx=time.monotonic()
            else:self.replies[kind]=data


class PressureLink:
    """气泵ACK等待独立线程执行，避免阻塞底盘500ms看门狗。"""
    def __init__(self, cfg, log):
        self.cfg, self.log = cfg, log
        self.lock = threading.Lock()
        self.value = Pressure()
        self.error = None
        self.pending = None
        self.ready = threading.Event()
        self.quit = threading.Event()
        self.thread = threading.Thread(target=self.run, daemon=True, name='pressure-link')
        self.on_ack = False

    def start(self):
        self.thread.start()
        if not self.ready.wait(5):
            raise RuntimeError('气泵串口初始化未完成')
        if self.error:
            raise RuntimeError(self.error)

    def command(self, on):
        with self.lock:
            self.pending = on
            self.on_ack = False

    def snapshot(self):
        with self.lock:
            if self.error:
                raise RuntimeError(self.error)
            return self.value

    def update(self, fields):
        # RSP中的code、SET_LINK等不代表压力新采样，不能给旧attach刷时间戳。
        if not all(k in fields for k in ('pump', 'attach')):
            return
        def truth(v):
            return str(v).lower() in ('1', 'true')
        with self.lock:
            pump, attached = truth(fields['pump']), truth(fields['attach'])
            state = fields.get('system_state', ('SUCTION_ATTACHED' if attached else 'SUCTION_FREE') if pump else 'RELEASE')
            self.value = Pressure(time.monotonic(), pump, attached, state, self.on_ack)

    def run(self):
        dev = None
        try:
            dev = AirGrip(self.cfg)
            # USB CDC刚打开时首条命令可能丢失；只在连接时等待DTR生效。
            if self.quit.wait(self.cfg.get('open_settle_s', .5)):
                return
            self.log('[AIRGRIP] '+str(dev.request('PING')))
            self.update(dev.request('GET_STATUS'))
            self.ready.set()
            last_link = last_poll = 0.0
            while not self.quit.is_set():
                with self.lock:
                    command, self.pending = self.pending, None
                if command is not None:
                    dev.set_pump(command)
                    with self.lock:
                        self.on_ack = command
                    # 明确在ACK之后再查询压力，不使用启动时的残余读数。
                    self.update(dev.request('GET_STATUS'))
                    self.log('[AIRGRIP] 泵'+('启动' if command else '释放')+'已确认')
                now = time.monotonic()
                if now-last_link >= self.cfg['link_refresh_s']:
                    dev.send('SET_LINK', 'connected=1')
                    last_link = now
                if now-last_poll >= self.cfg['status_poll_s']:
                    dev.send('GET_STATUS')
                    last_poll = now
                for frame in dev.receive():
                    if frame['cmd'] in ('RSP', 'EVT'):
                        self.update(parse_kv(frame['payload']))
                self.quit.wait(.01)
        except Exception as exc:
            with self.lock:
                self.error = str(exc)
        finally:
            self.ready.set()
            # 抓住后关闭程序不能自动松泵让物料掉落！释放只能由明确的Release执行。
            # 官方板端断线保留原状态，这里只关串口，不调用会PUMP_OFF的AirGrip.close。
            if dev is not None and dev.ser is not None:
                dev.ser.close()

    def close(self):
        self.quit.set()
        # 在任务工作线程中等待串口真正释放，关泵按钮随后才能建立新连接。
        self.thread.join()


class Motion:
    """仅维护底盘速度与反馈，不发送舵机查询或PWM。"""
    def __init__(self, config, root, cancelled, log):
        self.cfg, self.cancelled, self.log = config, cancelled, log
        self.hardware = load_config(device_config(config, root, 'hardware_config'))
        self.car = FeedbackChassis(self.hardware)
        self.slew = Slew(self.hardware)
        self.last = time.monotonic()
        self.integrals = [0.0, 0.0]
        try:
            self.car.connect()
            self.last = time.monotonic()
        except BaseException:
            self.car.close()
            raise

    @property
    def stopped(self):
        return abs(self.slew.v) < 1e-6 and abs(self.slew.w) < 1e-6

    def tick(self, v=0.0, w=0.0, immediate=False):
        if self.cancelled.is_set():
            raise InterruptedError('用户中断')
        now = time.monotonic()
        dt = now-self.last
        self.integrals[0] += self.slew.v*dt
        self.integrals[1] += self.slew.w*dt
        self.car.healthy()
        if immediate:
            self.slew.v = self.slew.w = 0.0
            v = w = 0.0
        else:
            v, w = self.slew.step(v, w, dt)
        self.car.speed(v, w)
        self.last = now
        self.cancelled.wait(1/self.hardware['control']['publish_hz'])

    def wait(self, duration, v=0.0, w=0.0):
        end = time.monotonic()+duration
        while time.monotonic() < end:
            self.tick(v, w)

    def settle(self):
        while not self.stopped:
            self.tick()
        self.wait(self.cfg['alignment']['settle_s'])

    def feedback(self):
        self.car.healthy()
        if time.monotonic()-self.car.last_body_rx>self.hardware['control']['telemetry_timeout_s']:
            raise RuntimeError('编码器/IMU反馈过期，停止移动')
        return self.car.pose.tuple(),self.car.body['vx'],self.car.body['wz']

    def wait_stationary(self):
        quiet=None
        while True:
            self.tick()
            pose,v,w=self.feedback()
            if abs(v)<=.01 and abs(w)<=.03:
                quiet=time.monotonic() if quiet is None else quiet
                if time.monotonic()-quiet>=.3:return pose
            else:quiet=None

    def close(self):
        self.car.close()
