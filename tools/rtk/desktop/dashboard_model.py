"""Pure presentation of recent RTK, registration and navigation observations."""
import math
from threading import Lock
import time


FIXES = {
    0: ('NO FIX', 'bad'), 1: ('SINGLE', 'warn'), 2: ('DGPS', 'warn'),
    4: ('FIXED', 'good'), 5: ('FLOAT', 'caution'),
}
REGISTRATION_STATES = {
    'unconfigured': '基准未配置', 'invalid': '未开始', 'collecting': '采集中',
    'ready': '配准完成', 'degraded': '降级',
}
REGISTRATION_REASONS = {
    'registration_required': '等待开始地理配准',
    'no_motion_between_stationary_windows': '等待直行采样',
    'need_stationary_start_and_end_GNSS_windows': '等待起点和终点静止采样',
    'RTK unavailable/stale': '等待新鲜固定解',
    'waiting_for_global_prediction_at_GNSS_stamp': '等待全局定位更新',
}


def split_screen(x, y, width, height):
    left = width * 3 // 4
    return (x, y, left, height), (x + left, y, width - left, height)


def as_count(value):
    try:
        count = int(value)
        return count if count >= 0 else None
    except (TypeError, ValueError, OverflowError):
        return None


def coordinates(value):
    def render(key, precision, unit=''):
        member = value.get(key)
        return f'{member:.{precision}f}{unit}' if isinstance(member, (int, float)) and math.isfinite(member) else '—'
    return (f'{render("latitude", 7)}, {render("longitude", 7)} | '
            f'X: {render("x", 2, "m")}, Y: {render("y", 2, "m")}')


class Dashboard:
    def __init__(self):
        self._lock = Lock()
        self._samples = {}

    def record(self, key, value, now=None):
        with self._lock:
            self._samples[key] = (time.monotonic() if now is None else now, value)

    def recent(self, key, now, age=2.5):
        with self._lock:
            sample = self._samples.get(key)
        return sample[1] if sample and 0 <= now - sample[0] <= age else None

    def latest_quality(self, now):
        with self._lock:
            samples = [self._samples[key] for key in ('quality', 'gnss') if key in self._samples]
        fresh = [sample for sample in samples if 0 <= now - sample[0] <= 3.]
        return max(fresh, key=lambda sample: sample[0])[1] if fresh else {}

    def planner_frame(self, now):
        return self.recent('frame', now, 2.) or self.recent('configured_frame', now, 1.5)

    def view(self, now):
        gnss = self.latest_quality(now)
        try:
            if float(gnss.get('gga_age_s', 0.)) > 3.:
                gnss = {}
        except (TypeError, ValueError):
            pass
        geo = self.recent('geographic', now, 1.5) or {}
        frame = self.planner_frame(now) or 'odom'
        fix, fix_color = FIXES.get(as_count(gnss.get('gga_quality')), FIXES[0])
        satellites = as_count(gnss.get('satellites'))
        satellite_text = '—' if satellites is None else f'{satellites} 颗'
        usable = str(gnss.get('fresh_rtk_usable')).lower() == 'true' and fix == 'FIXED'
        lidar = self.recent('lidar', now) is True
        lio = self.recent('lio', now) is True
        driver = self.recent('driver', now, 1.5)
        mux = self.recent('mux', now, 1.5) or {}
        state = geo.get('state')
        nav = state in ('ready', 'degraded') and geo.get('registered') is True and frame == 'map'
        if mux.get('emergency_stop_latched') is True:
            mux_label = '急停锁存', 'bad'
        elif mux.get('manual_hold') is True:
            mux_label = 'HOLD · 保持停车', 'caution'
        elif mux.get('manual_hold') is False:
            mux_label = '已放行', 'good'
        else:
            mux_label = '等待状态', 'neutral'
        reason = geo.get('reason')
        registration = REGISTRATION_STATES.get(state, '等待配准状态')
        if state == 'collecting':
            count = as_count(geo.get('registration_samples')) or 0
            movement = self.recent('registration_motion', now) or {}
            action = '等待 Fixed'
            if usable:
                action = (f'直行配准中 · 已移动 {movement.get("distance", 0.):.1f}m'
                          if movement.get('moving') else '静止采样中')
            registration += f' · {action} · {count} 个样本'
        elif state == 'ready' and not nav:
            registration += ' · 等待 map 导航'
        elif state in ('invalid', 'degraded') and not usable:
            registration += ' · 等待 Fixed'
        elif isinstance(reason, str) and reason:
            registration += ' · ' + REGISTRATION_REASONS.get(reason, reason)
        manual = self.recent('manual', now)
        route = self.recent('route', now) or {}
        points = route.get('points', [])
        return {
            'rtk': f'RTK状态：{fix}（卫星：{satellite_text}）',
            'fix_color': fix_color,
            'frame': frame,
            'navigation': nav,
            'degraded': state == 'degraded',
            'usable': usable,
            'registration': registration,
            'progress': 4 if nav else 3 if state in ('ready', 'collecting', 'degraded')
            else 2 if usable else 1 if lidar and lio else 0,
            'lidar': ('就绪', 'good') if lidar else ('等待数据', 'neutral'),
            'lio': ('就绪', 'good') if lio else ('等待数据', 'neutral'),
            'driver': ('就绪', 'good') if driver is True else
            ('未就绪', 'bad') if driver is False else ('等待状态', 'neutral'),
            'mux': mux_label,
            'destination': '正在前往：' + (manual or route.get('destination') or '待命'),
            'own': '自身坐标：' + coordinates(self.recent('own', now) or {}),
            'points': ['Point1：' + coordinates(points[0] if points else {}),
                       'Point2：棍木', 'Point3：棍木'],
        }


ACTIVE_ACTION_STATES = (0, 1, 6, 7)
ENDED_TASK_STATES = {'IDLE', 'COMPLETED', 'CANCELED', 'STOPPED', 'FAILED', 'SUCCEEDED'}


def task_in_progress(value):
    return bool(value and value.get('task_id') and value.get('phase') not in ENDED_TASK_STATES)


def manual_goal_block(frame, driver, mux, statuses, geographic, tasks, local_pending=False):
    if frame not in ('odom', 'map') or statuses is None:
        return '等待当前导航坐标系和 move_base 状态'
    if local_pending or any(status in ACTIVE_ACTION_STATES for _, status in statuses):
        return 'move_base 已有目标，当前目标优先'
    if any(task_in_progress(task) for task in tasks):
        return '空地任务正在执行或等待恢复'
    if driver is not True:
        return '底盘未就绪'
    if not mux or mux.get('manual_hold') is not False or mux.get('emergency_stop_latched') is not False:
        return 'Mux 保持停车或急停锁存，请使用原有操作流程'
    if geographic and geographic.get('state') == 'collecting':
        return '正在地理配准，等待配准动作结束'
    if frame == 'map' and (not geographic or geographic.get('state') != 'ready'
                           or geographic.get('registered') is not True):
        return '等待 map 导航配准就绪'
    return None
