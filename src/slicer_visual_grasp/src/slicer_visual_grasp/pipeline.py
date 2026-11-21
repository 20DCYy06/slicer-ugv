"""可中断的动作线程入口；UI线程只负责显示，绝不等待串口ACK。"""
import json
import math
from pathlib import Path
import threading
import time
import cv2
from .devices import Motion, PressureLink
from c30d_servo import Servo
from .paths import device_config


def save_json(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temp = path.with_suffix('.tmp')
    temp.write_text(json.dumps(value, ensure_ascii=False, indent=2)+'\n')
    temp.replace(path)


class Runtime:
    def __init__(self, cfg, root, camera, cancel, log):
        self.cfg, self.root, self.camera = cfg, root, camera
        self.cancel, self.log = cancel, log
        self.lowered_s = 0.0
        self.pump_lock = threading.Lock()

    def observation(self):
        frame, obs, _, error = self.camera.snapshot()
        self.last_observation_frame=frame
        if error:
            raise RuntimeError(error)
        if obs.center is None or time.monotonic()-obs.stamp > self.cfg['alignment']['observation_max_age_s']:
            raise RuntimeError('目标丢失或相机画面过期；停止本次操作，请重新选框')
        if self.cfg['alignment'].get('two_point',False) and obs.nozzle_center is None:
            raise RuntimeError('吸嘴模板未匹配；检查红色吸嘴框或模板区域配置')
        return obs



    def check_cancelled(self):
        if self.cancel.is_set():
            raise InterruptedError('用户中断')

    def pick(self):
        from .snapshot_move import make_plan,LockedMove
        if self.lowered_s > 0:
            raise RuntimeError('吸嘴尚有未回升的行程，请先点击释放')
        servo, max_lower_s = self.servo_settings()
        cfg = self.cfg['snapshot_move']
        if cfg['forward_image_direction'] not in ('up','down','left','right'):
            raise ValueError('请在config.json设置底盘正线速度的画面方向')
        self.check_cancelled()
        motion = Motion(self.cfg,self.root,self.cancel,self.log)
        try:
            self.log('[STATE] WAIT_STATIONARY：下发零速并等待实际反馈停稳')
            pose0=motion.wait_stationary()
            self.camera.set_active(True)
            observation=self.observation()  # 唯一用于规划的图像观测
            plan=make_plan(observation,self.cfg['camera'],cfg)
            plan['start_feedback_pose']=list(pose0)
            plan['nozzle_origin_px']=list(observation.nozzle_center)
            save_json(self.root/'data/last_move_plan.json',plan)
            if self.last_observation_frame is not None:
                cv2.imwrite(str(self.root/'data/last_move_snapshot.jpg'),self.last_observation_frame)
            self.log('[SNAPSHOT_LOCKED] pixel=%s vision_xy=%s extra=%.3fm body_xy=(%.4f,%.4f)m；运动中不重算视觉目标'%
                     (plan['pixel_vector'],plan['vision_body_xy_m'],plan['approach_extra_m'],*plan['body_xy_m']))
            mover=LockedMove(plan['body_xy_m'],pose0,cfg)
            previous=None;last_log=0.
            while mover.stage!='DONE':
                pose,v_measured,w_measured=motion.feedback()
                v,w=mover.step(pose)
                if previous!=mover.stage:
                    self.log('[STATE] '+mover.stage);previous=mover.stage
                motion.tick(v,w)
                if time.monotonic()-last_log>.5:
                    self.log('[MOVE_FEEDBACK] xy=(%.4f,%.4f)m yaw=%.3f command=(%.3f,%.3f) measured=(%.3f,%.3f)'%
                             (*pose,v,w,v_measured,w_measured));last_log=time.monotonic()
            self.log('[STATE] STOP_BEFORE_GRASP')
            motion.wait_stationary()
        finally:
            try:
                motion.close()
            finally:
                self.camera.set_active(False)

        with self.pump_lock:
            self.lower_and_grasp(servo, max_lower_s)

    def lower_and_grasp(self, servo, max_lower_s):
        self.check_cancelled()
        grasp = self.cfg['grasp']
        pcfg = json.loads(device_config(self.cfg, self.root, 'airgrip_config').read_text())
        pcfg['status_poll_s'] = grasp['status_poll_s']
        pcfg['open_settle_s'] = grasp['open_settle_s']
        pump = PressureLink(pcfg, self.log)
        requested = attached = False
        try:
            pump.start()
            self.check_cancelled()
            if pump.snapshot().attached:
                raise RuntimeError('开始前已有吸附信号，请先释放')
            self.log('[STATE] PUMP_START：底盘已停稳，开启抽气')
            requested_at = time.monotonic()
            requested = True
            pump.command(True)
            deadline = requested_at + 2 * pcfg['ack_timeout_s'] + grasp['status_max_age_s']
            while True:
                self.check_cancelled()
                pressure = pump.snapshot()
                now = time.monotonic()
                if (pressure.on_ack and pressure.pump
                        and requested_at <= pressure.stamp <= now
                        and now - pressure.stamp <= grasp['status_max_age_s']):
                    break
                if now >= deadline:
                    raise TimeoutError('未收到本次开泵的有效反馈')
                self.cancel.wait(.02)

            def grip_confirmed():
                value = pump.snapshot()
                if (not value.pump or not value.on_ack or value.stamp < requested_at
                        or time.monotonic() - value.stamp > grasp['status_max_age_s']):
                    raise RuntimeError('下降期间抽气或压力反馈中断')
                return value.attached and value.state == 'SUCTION_ATTACHED'

            self.log(f'[STATE] LOWER：下放吸嘴，最长 {max_lower_s:g} s')
            self.move_servo('down', max_lower_s, servo, grip_confirmed)
            self.check_cancelled()
            attached = True
            self.log(f'[DONE] 已吸附，已停止下降，记录下放时间 t={self.lowered_s:.3f} s')
        finally:
            pump.close(release=requested and not attached)

    def servo_settings(self):
        cfg = json.loads(device_config(self.cfg, self.root, 'servo_config').read_text())
        direction = self.cfg.get('servo_up_direction', 'clockwise')
        if direction not in ('clockwise', 'counterclockwise'):
            raise ValueError('舵机方向配置无效')
        highest = self.cfg['grasp'].get('max_lower_s', cfg['tap_duration_s'])
        cw, ccw = cfg['clockwise_us'], cfg['counterclockwise_us']
        if (type(cw) is not int or type(ccw) is not int or not 500 <= ccw < 1500 < cw <= 2500
                or cfg['neutral_us'] != 1500 or not 5 <= cfg['refresh_hz'] <= 100
                or not math.isfinite(highest) or highest <= 0):
            raise ValueError('舵机脉宽、中位、下降时限或刷新频率无效')
        up, down = (cw, ccw) if direction == 'clockwise' else (ccw, cw)
        return {'up': up, 'down': down, 'refresh_hz': cfg['refresh_hz']}, highest

    def move_servo(self, direction, duration, cfg=None, finished=None):
        if cfg is None:
            cfg, _ = self.servo_settings()
        self.check_cancelled()
        # A fresh confirmed attachment while already touching the object means t=0.
        if finished is not None and finished():
            return
        servo = Servo(device_config(self.cfg, self.root, 'hardware_config'))
        started = None
        try:
            servo.check(required_pin=14)
            while True:
                self.check_cancelled()
                if finished is not None and finished():
                    return
                now = time.monotonic()
                if started is None:
                    started = now
                elif now - started >= duration:
                    if finished is not None:
                        raise TimeoutError('到达下降时限，仍未收到吸附信号')
                    return
                servo.pulse(cfg[direction])
                servo.receive()
                remaining = duration - (time.monotonic() - started)
                self.cancel.wait(max(0., min(1 / cfg['refresh_hz'], remaining)))
        finally:
            elapsed = 0. if started is None else min(duration, time.monotonic() - started)
            try:
                servo.close()  # Send neutral before relinquishing C30D.
            finally:
                if direction == 'down':
                    self.lowered_s += elapsed
                else:
                    self.lowered_s = max(0., self.lowered_s - elapsed)

    def pump_off(self):
        cfg = json.loads(device_config(self.cfg, self.root, 'airgrip_config').read_text())
        from airgrip.keyboard import AirGrip
        with self.pump_lock:
            dev = AirGrip(cfg)
            try:
                time.sleep(self.cfg['grasp']['open_settle_s'])
                return dev.set_pump(False)
            finally:
                dev.ser.close()

    def release(self):
        self.log('[RELEASE] 气泵关闭：'+str(self.pump_off()))
        self.check_cancelled()
        if self.lowered_s > 0:
            self.log(f'[STATE] LIFT：按记录回升 {self.lowered_s:.3f} s')
            self.move_servo('up', self.lowered_s)
            self.log('[DONE] 释放和回升结束')
