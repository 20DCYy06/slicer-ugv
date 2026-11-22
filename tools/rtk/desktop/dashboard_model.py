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
NAV_STATUS_AGE = 1.5


def as_count(value):
    try:
        count = int(value)
        return count if count >= 0 else None
    except (TypeError, ValueError, OverflowError):
        return None


def coordinates(value):
    lat, lon = value.get('latitude'), value.get('longitude')
    if all(isinstance(member, (int, float)) and math.isfinite(member) for member in (lat, lon)):
        return f'{lat:.7f}, {lon:.7f}'
    return '—'


class StableStatus:
    """Debounce display changes without feeding values back into control checks."""
    def __init__(self, delay=.45):
        self.delay = delay
        self.shown = {}
        self.pending = {}

    def update(self, key, value, now, immediate=False):
        if immediate or key not in self.shown or self.shown[key] == value:
            self.pending.pop(key, None)
            self.shown[key] = value
        else:
            candidate, since = self.pending.get(key, (value, now))
            if candidate != value:
                since = now
            if now - since >= self.delay:
                self.shown[key] = value
                self.pending.pop(key, None)
            else:
                self.pending[key] = value, since
        return self.shown[key]


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
        # A callback may publish a receive-clock sample after the UI tick took its timestamp.
        return sample[1] if sample and now - sample[0] <= age else None

    def latest_quality(self, now):
        with self._lock:
            samples = [self._samples[key] for key in ('quality', 'gnss') if key in self._samples]
        fresh = [sample for sample in samples if now - sample[0] <= 3.]
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
        geo = self.recent('geographic', now, NAV_STATUS_AGE) or {}
        frame = self.planner_frame(now) or 'odom'
        fix, fix_color = FIXES.get(as_count(gnss.get('gga_quality')), FIXES[0])
        satellites = as_count(gnss.get('satellites'))
        satellite_text = '—' if satellites is None else f'{satellites} 颗'
        usable = str(gnss.get('fresh_rtk_usable')).lower() == 'true' and fix == 'FIXED'
        lidar = self.recent('lidar', now) is True
        lio = self.recent('lio', now) is True
        driver = self.recent('driver', now, NAV_STATUS_AGE)
        mux = self.recent('mux', now, NAV_STATUS_AGE) or {}
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
        return {
            'rtk': f'RTK状态：{fix}（卫星：{satellite_text}）',
            'fix_color': fix_color,
            'frame': frame,
            'navigation': nav,
            'degraded': state == 'degraded',
            'collecting': state == 'collecting',
            'usable': usable,
            'registration': registration,
            'progress': 4 if nav else 3 if state in ('ready', 'collecting', 'degraded')
            else 2 if usable else 1 if lidar and lio else 0,
            'lidar': ('就绪', 'good') if lidar else ('等待数据', 'neutral'),
            'lio': ('就绪', 'good') if lio else ('等待数据', 'neutral'),
            'driver': ('就绪', 'good') if driver is True else
            ('未就绪', 'bad') if driver is False else ('等待状态', 'neutral'),
            'mux': mux_label,
            'destination': '正在前往：' + (self.recent('destination', now) or '待命'),
            'own': '当前位置：' + coordinates(self.recent('own', now) or {}),
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
        return 'Mux 保持停车或急停锁存，请查看右栏 Mux 状态'
    if geographic and geographic.get('state') == 'collecting':
        return '正在地理配准，等待配准动作结束'
    if frame == 'map' and (not geographic or geographic.get('state') != 'ready'
                           or geographic.get('registered') is not True):
        return '等待 map 导航配准就绪'
    return None
