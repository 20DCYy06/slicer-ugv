"""纯状态机与图像雅可比控制：无串口、无相机，可用假反馈离线测试。"""
from dataclasses import dataclass
import numpy as np
from servo_math import servo_command


@dataclass(frozen=True)
class Observation:
    seq: int
    stamp: float
    center: object  # (u,v)，丢失时为 None
    nozzle_center: object = None
    nozzle_score: float = 0.0
    nozzle_box: object = None
    surface_quad: object = None


@dataclass(frozen=True)
class Pressure:
    stamp: float = float('-inf')
    pump: bool = False
    attached: bool = False
    state: str = 'UNKNOWN'
    on_ack: bool = False


@dataclass
class Output:
    v: float = 0.0
    w: float = 0.0
    servo: str = 'neutral'
    pump_on: bool = False
    immediate_stop: bool = False


def image_velocity(center, aim, jacobian, cfg):
    """J=[du/ds,du/dyaw; dv/ds,dv/dyaw]，实测方向，不能猜后置相机正负号。

    用一个共同缩放因子限速，保留图像纠偏方向；不对两分量分别粗暴截断。
    """
    result = servo_command(center, aim, jacobian, cfg['gain_per_s'],
                           cfg['max_linear_m_s'], cfg['max_angular_rad_s'], 0.0,
                           max_reverse=cfg['max_linear_m_s'])
    if result['status'] not in ('active', 'reached'):
        raise ValueError('视觉控制计算失败：'+result['status'])
    return result['vx'], result['wz']


def validate_calibration(data, camera):
    if data.get('resolution') != [camera['width'], camera['height']]:
        raise ValueError('标定分辨率与当前相机不一致，请重新标定')
    if data.get('device') != camera['device']:
        raise ValueError('相机设备变化，请重新标定')
    aim = np.asarray(data['aim_px'], float)
    matrix = np.asarray(data['jacobian'], float)
    if aim.shape != (2,) or matrix.shape != (2, 2) or not np.isfinite(matrix).all():
        raise ValueError('标定矩阵必须是有限的2×2矩阵')
    if not np.isfinite(aim).all() or not (0 <= aim[0] < camera['width'] and 0 <= aim[1] < camera['height']):
        raise ValueError('吸嘴对应像素不在图像内')
    if np.linalg.cond(matrix) > 50:
        raise ValueError('当前视角无法分别控制两个图像方向；调整相机俯角后重新标定')


class GraspLogic:
    """只做一次抓取。DONE 后仍保持泵，不自动释放、不自动重复抓取。"""
    def __init__(self, config, calibration, now):
        self.cfg = config
        self.aim = calibration['aim_px']
        self.jacobian = calibration['jacobian']
        self.state = 'ALIGN'
        self.reason = ''
        self.since = now
        self.stable = 0
        self.last_seq = -1
        self.pump_started = False
        self.lift_started = None

    def transition(self, state, now):
        self.state = state
        self.since = now

    def fail(self, reason, now):
        self.reason = reason
        self.transition('ERROR', now)
        return Output(immediate_stop=True)

    def step(self, now, obs, pressure, stopped=True):
        a, g = self.cfg['alignment'], self.cfg['grasp']
        if self.state in ('DONE', 'ERROR'):
            return Output(immediate_stop=True)
        # 只在视觉对准阶段要求目标可见。下降后吸嘴遮挡目标是正常现象。
        if self.state in ('ALIGN', 'STOP_BASE'):
            if obs.center is None or now - obs.stamp > a['observation_max_age_s']:
                return self.fail('目标丢失或相机画面过期，已停止，不会继续盲走', now)
            if a.get('two_point',False):
                if obs.nozzle_center is None:
                    return self.fail('吸嘴未识别，停止双点视觉对准',now)
                self.aim=np.asarray(obs.nozzle_center,float)
        if self.state == 'ALIGN':
            error = np.asarray(obs.center) - self.aim
            if np.max(np.abs(error)) <= a['tolerance_px']:
                if obs.seq != self.last_seq:
                    self.stable += 1
                if self.stable >= a['stable_frames']:
                    self.transition('STOP_BASE', now)
                out = Output()
            else:
                self.stable = 0
                out = Output(*image_velocity(obs.center, self.aim, self.jacobian, a))
            self.last_seq = obs.seq
            return out
        if self.state == 'STOP_BASE':
            if not stopped:
                return Output()
            # 停车缓降可能带来几像素位移：停车后再看同一目标，不带偏差下降。
            if np.max(np.abs(np.asarray(obs.center)-self.aim)) > a['tolerance_px']:
                self.stable = 0
                self.transition('ALIGN', now)
                return Output()
            if now-pressure.stamp > g['status_max_age_s']:
                return self.fail('吸口状态过期，未开始下降', now)
            if pressure.attached:
                return self.fail('开始前吸附信号已经为真，请先检查/释放残余负压', now)
            self.transition('PUMP_START', now)
            self.pump_started = True
            return Output(pump_on=True)
        if self.state in ('PUMP_START', 'LOWER', 'LIFT'):
            if now-pressure.stamp > g['status_max_age_s']:
                return self.fail('压力反馈中断，已停止升降；保持当前泵状态', now)
        if self.state == 'PUMP_START':
            # ACK只代表泵接受命令。必须等新的压力状态，再允许下降。
            if pressure.on_ack and pressure.pump and pressure.stamp >= self.since:
                self.transition('LOWER', now)
            return Output()
        if self.state == 'LOWER':
            if not pressure.pump:
                return self.fail('下降期间气泵未运行', now)
            if pressure.attached and pressure.state == 'SUCTION_ATTACHED':
                self.transition('LIFT', now)
                return Output()  # 先发中位，下一周期再上升
            if now-self.since >= g['max_lower_s']:
                return self.fail('达到配置的下降行程时间但尚未吸住，停止下降', now)
            return Output(servo='lower')
        if self.state == 'LIFT':
            if not pressure.pump or not pressure.attached:
                return self.fail('上升期间吸附信号丢失，停止升降', now)
            if self.lift_started is None:
                self.lift_started = now
            if now-self.lift_started >= g['lift_s']:
                self.transition('DONE', now)
                return Output()
            return Output(servo='lift')
        raise RuntimeError('未知状态 '+self.state)
