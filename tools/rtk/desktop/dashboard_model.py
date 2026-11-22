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
TASK_PHASES = {
    'WAITING_NAVIGATION': '等待导航就绪',
    'NAVIGATION_BUSY': '导航正在处理其他目标',
    'WAITING_RTK': '等待有效 RTK',
    'WAITING_GLOBAL': '等待全局定位与 TF',
    'WAITING_ODOM': '等待本地里程计',
    'WAITING_COSTMAP': '等待代价地图',
    'WAITING_STOP': '确认停车',
    'PLANNING_PREPARATION': '规划局部准备路线',
    'PREPARING': '自动准备运动',
    'CONFIRMING_REGISTRATION': '确认配准',
    'SWITCHING_NAVIGATION': '切换 map 导航',
    'STOPPING': '正在停车',
    'PAUSING': '正在暂停',
    'PAUSED': '已暂停',
    'CANCELING': '正在取消',
    'BLOCKED': '任务受阻',
    'FAILED': '任务失败',
    'NAVIGATING': '导航中',
    'VERIFYING': '停车及地理到点复核',
    'STARTING': '启动导航',
    'READY': '等待接续目标',
    'IDLE': '等待任务',
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

    def local_target(self):
        # Written when the desktop launches the client and cleared when it exits.
        with self._lock:
            sample = self._samples.get('wgs84')
        return sample[1] if sample is not None else None

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
        runtime = self.recent('runtime', now) or {}
        task = active_task(self, now)
        if runtime.get('busy'):
            phase, task_reason = runtime.get('phase'), runtime.get('reason')
        elif task:
            phase = task.get('runtime_phase') or task.get('phase')
            task_reason = task.get('last_error') or task.get('reason')
        elif any(state in ACTIVE_ACTION_STATES
                 for _, state in self.recent('action', now, NAV_STATUS_AGE) or ()):
            phase, task_reason = 'NAVIGATING', None
        elif self.local_target() is not None:
            phase, task_reason = 'READY', None
        elif runtime.get('phase') in ('BLOCKED', 'FAILED', 'STARTING'):
            phase, task_reason = runtime['phase'], runtime.get('reason')
        elif self.recent('wgs84_failure', now, 30.):
            phase, task_reason = 'FAILED', None
        else:
            phase, task_reason = 'IDLE', None
        task_phase = TASK_PHASES.get(phase, phase or '等待任务')
        if task_reason and task_reason != phase:
            task_phase += ' · ' + REGISTRATION_REASONS.get(task_reason, str(task_reason))
        metrics = []
        if runtime.get('busy') or (phase in ('BLOCKED', 'FAILED') and runtime.get('phase') == phase):
            for key, title, precision in (('distance_m', '路程', 1), ('span_m', '展开', 1),
                                           ('rmse_m', '残差', 2)):
                value = runtime.get(key)
                if isinstance(value, (float, int)) and math.isfinite(value):
                    metrics.append(f'{title} {value:.{precision}f} m')
        reason = geo.get('reason')
        registration = REGISTRATION_STATES.get(state, '等待配准状态')
        if state == 'collecting':
            count = as_count(geo.get('registration_samples')) or 0
            action = '配对样本' if usable else '等待 Fixed'
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
            'task_phase': '任务阶段：' + task_phase,
            'task_metrics': ' · '.join(metrics),
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


def active_task(board, now):
    for protocol in ('v1', 'v2'):
        value = board.recent(protocol, now)
        if task_in_progress(value):
            return value
    return None


def task_destination(board, now):
    task = active_task(board, now)
    if task:
        point = task.get('final_goal') or task.get('target')
        if isinstance(point, dict):
            point = {'latitude': point.get('latitude', point.get('lat')),
                     'longitude': point.get('longitude', point.get('lon'))}
            text = coordinates(point)
            if text != '—':
                return text
    point = board.local_target()
    if point is not None:
        return coordinates(point)
    runtime = board.recent('runtime', now) or {}
    point = runtime.get('target')
    if runtime.get('busy') and isinstance(point, dict):
        text = coordinates(point)
        if text != '—':
            return text
    return None


def manual_goal_block(frame, driver, mux, statuses, geographic, tasks, desktop_goals=(), runtime=None):
    if runtime and runtime.get('busy'):
        return '正在自动准备地理航行，等待配准或模式切换结束'
    if frame not in ('odom', 'map') or statuses is None:
        return '等待当前导航坐标系和 move_base 状态'
    if any(status in ACTIVE_ACTION_STATES and goal not in desktop_goals for goal, status in statuses):
        return 'move_base 有其他控制端的目标，当前目标优先'
    if any(task_in_progress(task) for task in tasks):
        return '空地任务正在执行或等待恢复'
    if driver is not True:
        return '底盘未就绪'
    if not mux or mux.get('manual_hold') is not False or mux.get('emergency_stop_latched') is not False:
        return 'Mux 保持停车或急停锁存，请查看右栏 Mux 状态'
    if mux.get('teleop_fresh') is True or mux.get('selected_source') == 'mission':
        return '其他速度控制源正在接管'
    if geographic and geographic.get('state') == 'collecting':
        return '正在地理配准，等待配准动作结束'
    if frame == 'map' and (not geographic or geographic.get('state') != 'ready'
                           or geographic.get('registered') is not True):
        return '等待 map 导航配准就绪'
    return None
