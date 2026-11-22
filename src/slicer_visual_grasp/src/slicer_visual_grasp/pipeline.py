"""可中断的动作线程入口；UI线程只负责显示，绝不等待串口ACK。"""
import json
import math
import os
from pathlib import Path
import threading
import time
import cv2
from .devices import Motion, PressureLink
from c30d_servo import Servo
from .paths import device_config


POST_GRASP_LIFT_S = 2.0
TARGET_WAIT_S = 5.0


def save_json(path, value, *, durable=False):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temp = path.with_suffix('.tmp')
    with temp.open('w', encoding='utf-8') as stream:
        stream.write(json.dumps(value, ensure_ascii=False, indent=2)+'\n')
        if durable:
            stream.flush()
            os.fsync(stream.fileno())
    temp.replace(path)
    if durable:
        fd = os.open(path.parent, os.O_RDONLY | os.O_DIRECTORY)
        try:
            os.fsync(fd)
        finally:
            os.close(fd)


class Runtime:
    def __init__(self, cfg, root, camera, cancel, log):
        self.cfg, self.root, self.camera = cfg, root, camera
        self.cancel, self.log = cancel, log
        self.lowered_s = 0.0
        self.holding = False
        self.pump_needs_release = False
        self.travel_unknown = False
        self.recovery_path = None
        self.pump_lock = threading.Lock()

    @property
    def needs_release(self):
        return self.holding or self.pump_needs_release or self.travel_unknown or self.lowered_s > 0

    def load_recovery(self):
        self.recovery_path = self.root / 'data/grasp_recovery.json'
        try:
            state = json.loads(self.recovery_path.read_text(encoding='utf-8'))
            lowered = state['lowered_s']
            flags = ('holding', 'pump_needs_release', 'travel_unknown')
            if (type(lowered) not in (float, int) or not math.isfinite(lowered) or lowered < 0
                    or any(type(state[key]) != bool for key in flags)):
                raise ValueError('无效的恢复记录')
            self.lowered_s = lowered
            self.holding, self.pump_needs_release, self.travel_unknown = (state[key] for key in flags)
        except FileNotFoundError:
            return
        except (OSError, ValueError, KeyError, TypeError, OverflowError) as exc:
            self.pump_needs_release = self.travel_unknown = True
            self.log('[ERROR] 无法读取视觉抓取恢复记录：'+str(exc))

    def checkpoint(self, *, travel_unknown=None):
        if self.recovery_path is not None:
            save_json(self.recovery_path, {
                'lowered_s': self.lowered_s,
                'holding': self.holding,
                'pump_needs_release': self.pump_needs_release,
                'travel_unknown': self.travel_unknown if travel_unknown is None else travel_unknown,
            }, durable=True)

    def observation(self, since=0.):
        frame, obs, _, error = self.camera.snapshot()
        self.last_observation_frame=frame
        if error:
            raise RuntimeError(error)
        if (obs.center is None or obs.stamp < since
                or time.monotonic()-obs.stamp > self.cfg['alignment']['observation_max_age_s']):
            raise RuntimeError('目标丢失或相机画面过期；停止本次操作，请重新选框')
        if self.cfg['alignment'].get('two_point',False) and obs.nozzle_center is None:
            raise RuntimeError('吸嘴模板未匹配；检查红色吸嘴框或模板区域配置')
        return obs

    def wait_for_observation(self, since):
        deadline = since + TARGET_WAIT_S
        while True:
            self.check_cancelled()
            try:
                observation = self.observation(since)
                if observation.nozzle_center is None or observation.surface_quad is None:
                    raise RuntimeError('未识别到固定吸嘴原点和物体四角')
                return
            except RuntimeError as exc:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError('等待新相机目标超时：'+str(exc)) from exc
                self.cancel.wait(min(.05, remaining))

    def check_cancelled(self):
        if self.cancel.is_set():
            raise InterruptedError('用户中断')

    def pick(self, *, wait_for_target=False):
        started = time.monotonic()
        if self.needs_release:
            raise RuntimeError('还有持物或未回升的行程，请先点击释放')
        servo, max_lower_s = self.servo_settings()
        cfg = self.cfg['snapshot_move']
        if cfg['forward_image_direction'] not in ('up','down','left','right'):
            raise ValueError('请在config.json设置底盘正线速度的画面方向')
        if wait_for_target:
            self.log('[STATE] WAIT_TARGET：等待本次抓取的新相机画面')
            self.wait_for_observation(started)
        grasp = self.cfg['grasp']
        pcfg = json.loads(device_config(self.cfg, self.root, 'airgrip_config').read_text())
        pcfg['status_poll_s'] = grasp['status_poll_s']
        pcfg['open_settle_s'] = grasp['open_settle_s']
        with self.pump_lock:
            self.check_cancelled()
            pump = PressureLink(pcfg, self.log)
            try:
                pump.start()
                self.require_idle_pump(pump)
                self.move_to_target(pump, started if wait_for_target else 0.)
                self.require_idle_pump(pump)
            except BaseException:
                pump.close()
                raise
            self.lower_and_grasp(pump, pcfg['ack_timeout_s'], servo, max_lower_s)

    def require_idle_pump(self, pump):
        pressure = pump.snapshot()
        if pressure.pump or pressure.attached:
            self.pump_needs_release = True
            self.checkpoint()
            raise RuntimeError('开始前已有抽气或吸附信号，请先释放')
        if time.monotonic() - pressure.stamp > self.cfg['grasp']['status_max_age_s']:
            raise RuntimeError('气泵状态反馈过期')

    def move_to_target(self, pump, since):
        from .snapshot_move import make_plan, LockedMove
        cfg = self.cfg['snapshot_move']
        self.check_cancelled()
        motion = Motion(self.cfg,self.root,self.cancel,self.log)
        try:
            self.log('[STATE] WAIT_STATIONARY：下发零速并等待实际反馈停稳')
            pose0=motion.wait_stationary()
            self.camera.set_active(True)
            observation=self.observation(since)  # 唯一用于规划的图像观测
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
                self.require_idle_pump(pump)
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

    def lower_and_grasp(self, pump, ack_timeout, servo, max_lower_s):
        grasp = self.cfg['grasp']
        requested = attached = False
        try:
            self.check_cancelled()
            self.log('[STATE] PUMP_START：底盘已停稳，开启抽气')
            self.pump_needs_release = True
            self.checkpoint()
            requested_at = time.monotonic()
            requested = True
            pump.command(True)
            deadline = requested_at + 2 * ack_timeout + grasp['status_max_age_s']
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
            attached = self.holding = True
            self.pump_needs_release = False
            self.checkpoint()
            lowered_s = self.lowered_s
            self.log(f'[STATE] LIFT_AFTER_GRASP：已吸附，下放 t={lowered_s:.3f} s，回升 {POST_GRASP_LIFT_S:g} s')
            self.move_servo('up', POST_GRASP_LIFT_S, servo)
            if not grip_confirmed():
                raise RuntimeError('抬升结束时未确认仍然吸附，请释放后重试')
            self.log(f'[DONE] 抓取后抬升结束，释放时剩余回升 {self.lowered_s:.3f} s')
        finally:
            if pump.close(release=requested and not attached):
                self.holding = self.pump_needs_release = False
                self.checkpoint()

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
        movement_recorded = False
        try:
            servo.check(required_pin=14)
            while True:
                self.check_cancelled()
                if finished is not None and finished():
                    return
                now = time.monotonic()
                if started is None:
                    # A killed process cannot account for travel before the watchdog stops the servo.
                    self.checkpoint(travel_unknown=True)
                    movement_recorded = True
                    self.check_cancelled()
                    started = time.monotonic()
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
            except Exception:
                if started is not None and self.recovery_path is not None:
                    self.travel_unknown = True
                raise
            finally:
                if direction == 'down':
                    self.lowered_s += elapsed
                else:
                    self.lowered_s = max(0., self.lowered_s - elapsed)
                if movement_recorded:
                    self.checkpoint()

    def pump_off(self):
        cfg = json.loads(device_config(self.cfg, self.root, 'airgrip_config').read_text())
        from airgrip.keyboard import AirGrip
        with self.pump_lock:
            dev = AirGrip(cfg)
            try:
                time.sleep(self.cfg['grasp']['open_settle_s'])
                result = dev.set_pump(False)
                self.holding = self.pump_needs_release = False
                self.checkpoint()
                return result
            finally:
                dev.ser.close()

    def release(self):
        if self.recovery_path is not None:
            self.pump_needs_release = True
        try:
            result = self.pump_off()
        except Exception:
            self.checkpoint()
            raise
        self.log('[RELEASE] 气泵关闭：'+str(result))
        self.check_cancelled()
        if self.travel_unknown:
            raise RuntimeError(f'升降行程未知，已关泵。请手动恢复初始高度后清除 {self.recovery_path}')
        if self.lowered_s > 0:
            self.log(f'[STATE] LIFT：按记录回升 {self.lowered_s:.3f} s')
            self.move_servo('up', self.lowered_s)
            self.log('[DONE] 释放和回升结束')
