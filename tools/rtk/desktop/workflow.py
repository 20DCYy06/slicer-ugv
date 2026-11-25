"""Orchestrate the existing launch files on the user's ROS master."""
from collections import deque
import json
import os
from pathlib import Path
import shlex
import sys
import time

from dashboard_model import ACTIVE_ACTION_STATES, task_in_progress
from owned_process import OwnedProcess as ProcessGroup
from workflow_discovery import Graph, NAV_ACTION

STEPS = ('rtk', 'sensors', 'driver', 'vision', 'navigation', 'air', 'wgs84', 'calibrate', 'map')
STOP_ORDER = ('wgs84', 'grasp_hold', 'grasp_unhold', 'map_release', 'map', 'prepare', 'calibrate',
              'air', 'navigation', 'vision', 'driver', 'sensors', 'rtk')
NODE_GROUPS = {
    'rtk': {'/geographic', '/cup_gnss', '/global_localization/ekf'},
    'sensors': {'/slicer_cloud_to_scan', '/slicer_ekf'},
    'driver': {'/slicer_c30d_driver'},
}
AUX = ('map_release', 'calibration_hold', 'grasp_hold', 'grasp_unhold', 'map', 'prepare')
CONTROL = ('rosrun', 'ugv_slicer_control', 'control.py')
SERIAL_GROUP = Path(__file__).resolve().parents[2] / 'manual' / 'serial_group.sh'
CONTROL_SOURCE = Path(__file__).resolve().parents[3] / 'src/ugv_slicer_control/scripts/control.py'
NAVIGATION = ('roslaunch', 'ugv_bringup', 'slicer_navigation_runtime.launch')
SWITCH_MAP = ('rosservice', 'call', '/geographic/navigation_map')
SWITCH_PREPARATION = ('rosservice', 'call', '/geographic/navigation_prepare')
ROUNDTRIP = ('rosrun', 'ugv_navigation', 'geographic.py', 'initialize',
             '--distance', '2.0', '--return-to-start')
VISUAL_GRASP = ('rosrun', 'slicer_visual_grasp', 'run.sh', '--run', '--headless')
DRIVER_COMMAND = ('roslaunch', 'ugv_bringup', 'slicer_body_driver.launch')
# The process runner escalates shutdown at 32 s. Allow another ROS graph poll.
HANDOFF_TIMEOUT = 35.
# Calibration hold has a 12 s process timeout. Allow discovery of its exit.
MAP_HANDOFF_TIMEOUT = 15.
AIR_PEER_TOPICS = {'v1': '/uav/test', 'v2': '/air_ground/v2/uav_state'}
GRASP_STATES = {'idle', 'busy', 'held', 'recovery', 'error'}


class OwnedProcess(ProcessGroup):
    def __init__(self, key, argv, env):
        if key in ('rtk', 'sensors', 'driver'):
            # The setuid sg helper strips the loader path; restore it for the ROS binaries.
            if 'LD_LIBRARY_PATH' in env:
                argv = ('env', f'LD_LIBRARY_PATH={env["LD_LIBRARY_PATH"]}', *argv)
            argv = ('bash', '-c', 'source "$1" && shift && run_serial_tool "$@"',
                    'rtk-desktop', str(SERIAL_GROUP), *argv)
        super().__init__(key, argv, env, log_prefix='rtk-desktop', stdin=key == 'vision')


class Workflow:
    def __init__(self, board, spawn=OwnedProcess, env=None, clock=time.monotonic):
        self.board = board
        self.spawn = spawn
        self.env = dict(os.environ if env is None else env)
        # The RViz input namespace stays private. The launched stack uses its existing names.
        self.env.pop('ROS_NAMESPACE', None)
        self.clock = clock
        self.graph = Graph()
        self.grasp_state = 'idle'
        self.grasp_command = None
        self.grasp_recovery = False
        self.grasp_handoff = None
        self.grasp_mux = None
        self.grasp_unpark = None
        self.handoff_deadline = 0.
        self.owned = {}
        self.started = {}
        self.stopping = set()
        self.exited = {}
        self.messages = deque(maxlen=3)
        self.calibration_hold = None
        self.calibration_hold_exited_at = None
        self.calibrating = False
        self.startup_calibration = None
        self.prepare_started = 0.
        self.map_handoff_deadline = 0.
        self.map_handoff_failure = ''
        self.odom_cruise = False
        self.stopping_all = False

    def recent(self, key):
        return self.board.recent(key, self.clock(), 2.5)

    def report(self, message):
        self.messages.append(message)
        print(f'RTK 流程 · {message}', flush=True)

    def present(self, key):
        graph = self.graph
        if key == 'air':
            return bool(graph.air_mode or graph.processes.get('air'))
        if key == 'navigation':
            return self.nav_present
        if key == 'map':
            return graph.frame == 'map' and NAV_ACTION in graph.nodes
        if key == 'wgs84':
            return False
        if key == 'vision':
            return bool(graph.processes.get(key))
        if key == 'calibrate':
            return bool(graph.processes.get(key) or (self.geo.get('state') == 'collecting' and
                        any(node.rsplit('/', 1)[-1].startswith('slicer_geographic_client_')
                            for node in graph.nodes)))
        return bool(graph.processes.get(key) or NODE_GROUPS[key] & graph.nodes)

    @property
    def geo(self):
        return self.recent('geographic') or {}

    @property
    def mux(self):
        return self.recent('mux') or {}

    @property
    def runtime(self):
        return self.recent('runtime') or {}

    @property
    def nav_present(self):
        return bool('navigation' in self.owned or self.runtime
                    or any(self.graph.processes.get(key) for key in ('navigation', 'odom', 'map', 'nav'))
                    or {NAV_ACTION, '/slicer_nav_arc_guard', '/slicer_velocity_mux'} & self.graph.nodes
                    or self.graph.publishers.get('/cmd_vel'))

    @property
    def startup_calibration_pending(self):
        return self.startup_calibration in ('preparing', 'running', 'awaiting_map', 'switching')

    @property
    def switching(self):
        return ('map' in self.owned or 'map_release' in self.owned
                or 'prepare' in self.owned or self.runtime.get('phase') == 'SWITCHING_NAVIGATION')

    @property
    def vision_driver_busy(self):
        return ('driver' in self.owned or self.present('driver') or self.ready('driver')
                or bool(self.graph.publishers.get('/wheel/odom')))

    @property
    def air_local_modes(self):
        return tuple(mode for mode in (self.graph.air_mode or '').split('+')
                     if mode and self.recent(mode) is not None)

    def _navigation_connected(self):
        return (self.graph.online and self.graph.frame in ('odom', 'map')
                and self.recent('action') is not None
                and NAV_ACTION in self.graph.subscribers.get(NAV_ACTION + '/goal', ()))

    def ready(self, key):
        if key == 'vision':
            return self.grasp_state == 'held' and 'vision' in self.owned
        if not self.graph.online:
            return False
        if key == 'rtk':
            return self.ready('calibrate') and self.board.fixed_position(self.clock()) is not None
        if key == 'sensors':
            return ((key in self.owned or self.present(key))
                    and bool(self.recent('lidar') and self.recent('odom') is not None))
        if key == 'driver':
            ready = self.recent('driver')
            return (bool(self.owned.get(key)) or self.present(key)) and (
                ready is True or (ready is None and self.recent('wheel_odom') is True))
        if key in ('navigation', 'map'):
            return self._navigation_connected() and (
                key == 'navigation' if self.graph.frame == 'odom' else self.ready('calibrate'))
        if key == 'wgs84':
            return False
        if key == 'calibrate':
            return self.geo.get('state') == 'ready' and self.geo.get('registered') is True
        if key == 'air':
            return any((mode == 'v1' and self.recent('aircraft_v1_command') is True)
                       or self.recent('aircraft_' + mode) in
                       self.graph.publishers.get(AIR_PEER_TOPICS[mode], ())
                       for mode in self.air_local_modes)

    def _waiting_status(self, key):
        if key == 'rtk':
            return '等待 RTK Fixed' if self.board.fixed_position(self.clock()) is None else '等待配准'
        if key == 'sensors':
            return '等待雷达扫描' if not self.recent('lidar') else '等待定位数据'
        if key == 'driver':
            return '等待底盘遥测'
        if key in ('navigation', 'map') and self.recent('odom') is None:
            return '等待本地里程计'
        if key == 'vision':
            return '外部抓取进程'
        if key == 'air':
            return '车端已启动 · 等飞机' if self.air_local_modes else '等待车端状态'
        return '等待就绪'

    @property
    def states(self):
        result = {}
        for key in STEPS:
            owned = key in self.owned
            external = self.present(key) and not owned
            if key in self.stopping:
                result[key] = ('yellow', '停止中', False)
            elif key == 'rtk' and self.startup_calibration:
                state = self.startup_calibration
                if state == 'awaiting_map':
                    reason = self._map_block() or '等待切换 map'
                    status = '配准已完成 · ' + reason.rstrip('。')
                else:
                    status = {
                        'waiting': self._roundtrip_block() or '即将往返配准',
                        'preparing': '正在切换配准档',
                        'running': '配准完成 · 正在返回起点' if self.ready('calibrate') else '正在往返配准',
                        'switching': '配准已完成 · 正在切换 map',
                        'map_failed': '配准已完成 · ' + self.map_handoff_failure.rstrip('。'),
                        'failed': '自动配准未完成',
                    }[state]
                result[key] = ('yellow', status, not self.stopping_all)
            elif key == 'map' and self.switching:
                result[key] = ('yellow', '切换中', False)
            elif key == 'wgs84':
                if owned:
                    result[key] = ('yellow', '取消本地任务', not self.stopping_all)
                else:
                    blocked = self._wgs84_block(self.board.fixed_position(self.clock()))
                    result[key] = ('yellow', blocked, False) if blocked else ('green', '可发送', True)
            elif key == 'calibrate' and (self.calibrating or self.present(key)):
                result[key] = ('yellow', '配准中', False)
            elif key == 'driver' and self.grasp_handoff in ('to_grasp', 'grasp', 'to_driver'):
                text = '正在恢复驱动' if self.grasp_handoff == 'to_driver' else '串口交给视觉抓取'
                result[key] = ('yellow', text, False)
            elif key == 'vision' and self.grasp_handoff in ('parking', 'parked', 'to_grasp'):
                text = '等待串口' if self.grasp_handoff == 'to_grasp' else '等待停车确认'
                if self.grasp_state in ('held', 'recovery') or self.grasp_recovery:
                    text = '释放' + text
                result[key] = ('yellow', text, False)
            elif key == 'vision' and self.grasp_state == 'held' and owned:
                result[key] = ('green', '抓取完成', self.grasp_handoff is None and not self.stopping_all)
            elif key == 'vision' and owned:
                color, status = {
                    'idle': ('red', '待抓取'), 'starting': ('yellow', '正在启动抓取'),
                    'busy': ('yellow', '释放中' if self.grasp_command == 'release' else '抓取中'),
                    'held': ('green', '抓取完成'), 'recovery': ('yellow', '需释放'), 'error': ('red', '抓取失败 · 重试'),
                }[self.grasp_state]
                result[key] = (color, status, self.grasp_state not in ('starting', 'busy')
                               and not self.stopping_all)
            elif key == 'vision' and not external and self.vision_driver_busy and 'driver' not in self.owned:
                result[key] = ('yellow', '底盘由外部终端占用', not self.stopping_all)
            elif key == 'vision' and not external and self.grasp_recovery:
                result[key] = ('yellow', '需释放', not self.stopping_all)
            elif self.ready(key):
                status = {'driver': '已完成', 'calibrate': '已配准',
                          'air': '收到对端消息'}.get(key, '已就绪')
                if key == 'navigation':
                    if external:
                        status = '外部运行'
                    else:
                        status = 'RTK 与 Nav Goal 就绪' if self.graph.frame == 'map' else 'Nav Goal 就绪'
                result[key] = ('green', status, key != 'calibrate' and not self.stopping_all)
            elif owned or external:
                result[key] = ('yellow', self._waiting_status(key), not self.stopping_all)
            else:
                result[key] = ('red', self.exited.get(key, ''), not self.stopping_all)
        states = [result[key] for key in ('sensors', 'driver')]
        if any(key in self.stopping for key in ('sensors', 'driver')):
            result['base'] = ('yellow', '停止中', False)
        elif all(color == 'green' for color, _, _ in states):
            result['base'] = ('green', '雷达和底盘就绪', not self.stopping_all)
        elif any(color != 'red' for color, _, _ in states):
            waiting = '等待雷达' if states[0][0] != 'green' else '等待底盘'
            result['base'] = ('yellow', waiting, not self.stopping_all)
        else:
            result['base'] = ('red', '', not self.stopping_all)
        return result

    def _grasp_click(self):
        if self.grasp_state in ('starting', 'busy'):
            return
        command = 'release' if self.grasp_state in ('held', 'recovery') else 'pick'
        try:
            self.owned['vision'].send(command)
        except (OSError, ValueError):
            self.report('抓取进程无法接收命令，请检查运行日志。')
        else:
            self.grasp_state = 'busy'
            self.grasp_command = command

    def _start_grasp(self):
        argv = VISUAL_GRASP
        if not self.grasp_recovery:
            argv += ('--pick-on-start',)
        if not self._start('vision', argv):
            return False
        self.grasp_state = 'starting'
        self.grasp_command = 'recover' if self.grasp_recovery else 'pick'
        return True

    def _request_grasp(self):
        if (self.runtime.get('busy') or 'wgs84' in self.owned or self.switching
                or self.calibrating or self.startup_calibration == 'awaiting_map'
                or any(key in self.owned for key in AUX)):
            self.report('请等待当前停车、配准或导航切换操作结束。')
            return
        if self.vision_driver_busy and 'driver' not in self.owned:
            self.report('底盘串口由外部终端占用，请在原终端停止驱动。')
            return
        if 'driver' not in self.owned:
            if self.nav_present:
                self.report('请等待底盘驱动和导航就绪。')
            else:
                self._run_grasp()
            return
        if not self._grasp_still():
            self.report('等待底盘驱动就绪并确认车辆停稳。')
            return
        self.grasp_mux = None
        if self.nav_present:
            mux = self.mux
            instance, generation = self._mux_key(mux)
            if not (instance and type(generation) is int and self._grasp_nav_idle()):
                self.report('请等待导航空闲、车辆停稳后再操作抓取。')
                return
            if mux['manual_hold'] is False:
                if self._move_control('grasp_hold', 'hold', '--require-idle',
                                      '--expected-mux-instance', instance,
                                      '--expected-generation', str(generation)):
                    self._set_grasp_handoff('parking')
                return
            self.grasp_mux = instance, generation
        self._set_grasp_handoff('to_grasp')
        self._signal_stop('driver')

    def _run_grasp(self):
        if 'vision' in self.owned:
            self._grasp_click()
            return self.grasp_state == 'busy'
        return self._start_grasp()

    def _grasp_still(self):
        now = self.clock()
        return (self.graph.online and 'driver' not in self.stopping
                and self.board.recent('driver', now, 1.) is True
                and self.board.recent('wheel_still', now, 1.) is True)

    @staticmethod
    def _mux_key(mux):
        return mux.get('mux_instance_id'), mux.get('takeover_generation')

    def _grasp_parked(self):
        if not self.nav_present and self.grasp_mux is None:
            return True
        mux = self.mux
        return (self.grasp_mux == self._mux_key(mux) and mux.get('manual_hold') is True
                and self._grasp_nav_idle())

    def _grasp_nav_idle(self):
        return (self.graph.online
                and self.graph.publishers.get('/cup_nav/manual_state') == {'/slicer_velocity_mux'}
                and self._safe_to_move() and self._nav_idle())

    def _set_grasp_handoff(self, phase):
        self.grasp_handoff = phase
        self.handoff_deadline = self.clock() + HANDOFF_TIMEOUT

    def _advance_grasp_handoff(self, fresh_graph):
        phase = self.grasp_handoff
        if (phase == 'grasp' and not self._grasp_parked()
                and 'vision' in self.owned and 'vision' not in self.stopping):
            self._signal_stop('vision')
            self.report('导航停车状态变化，已中止抓取动作。')
        if phase == 'parked':
            if fresh_graph and self._grasp_parked() and self._grasp_still():
                self._set_grasp_handoff('to_grasp')
                self._signal_stop('driver')
            elif self.clock() > self.handoff_deadline:
                self._set_grasp_handoff('to_driver')
                self.report('等待车辆停稳超时，已取消抓取。')
            return
        if phase not in ('to_grasp', 'to_driver'):
            return
        if self.clock() > self.handoff_deadline:
            if phase == 'to_grasp':
                self._set_grasp_handoff('to_driver')
                self.report('等待串口超时，已取消抓取，等待恢复底盘驱动。')
            else:
                self.grasp_handoff = None
                self.report('恢复底盘驱动超时，请检查串口占用后重新启动第 2 步。')
            return
        if not fresh_graph or not self.graph.online:
            return
        if phase == 'to_grasp':
            if not self._grasp_parked():
                self._set_grasp_handoff('to_driver')
                self.report('导航状态变化，已取消抓取，正在恢复底盘驱动。')
            elif not self.vision_driver_busy and not (self.present('vision') and 'vision' not in self.owned):
                self._set_grasp_handoff('grasp' if self._run_grasp() else 'to_driver')
            return
        if 'vision' in self.stopping or (self.present('vision') and 'vision' not in self.owned):
            return
        if 'driver' not in self.owned:
            if self.vision_driver_busy:
                return
            if not self._start('driver', DRIVER_COMMAND):
                self.grasp_handoff = None
                return
            self.report('底盘串口已交还，等待驱动就绪。')
        if self.grasp_unpark is None or self.grasp_state in ('busy', 'recovery'):
            self.grasp_handoff = None
        elif self.mux and (self.grasp_unpark != self._mux_key(self.mux) or not self._safe_to_move()):
            self.grasp_unpark = None
            self.grasp_handoff = None
        elif self._grasp_still() and self._grasp_nav_idle():
            instance, generation = self.grasp_unpark
            if self._move_control('grasp_unhold', 'release', '--expected-mux-instance', instance,
                                  '--expected-generation', str(generation)):
                self.grasp_handoff = 'unparking'

    def _grasp_log(self, lines, running):
        for line in lines:
            marker, _, state = line.partition(' ')
            if marker != 'GRASP_STATE' or state not in GRASP_STATES:
                continue
            self.grasp_state = state
            if state in ('idle', 'recovery') and self.grasp_command == 'recover':
                self.grasp_state = 'recovery'
                if running and not self.stopping_all and 'vision' not in self.stopping:
                    self._grasp_click()
                continue
            if state in ('held', 'recovery') and self.grasp_handoff == 'grasp':
                # A headless result follows the motion and servo cleanup. Its idle
                # process may stay alive to accept release without owning C30D.
                self._set_grasp_handoff('to_driver')
            if state == 'idle' and self.grasp_command == 'release':
                self.grasp_recovery = False
                self.exited['vision'] = '已释放'
                if running:
                    self._signal_stop('vision')
            if state == 'error':
                self.exited['vision'] = '抓取失败 · 重试'
                if running:
                    self._signal_stop('vision')
            if state != 'busy':
                self.grasp_command = None

    def _start(self, key, argv):
        try:
            self.owned[key] = self.spawn(key, argv, self.env)
        except (OSError, ValueError) as error:
            if key in STEPS:
                self.exited[key] = '启动失败'
            self.report(f'{key} 启动失败 · {error}')
            return False
        self.exited.pop(key, None)
        self.started[key] = self.clock()
        self.report(shlex.join(argv))
        return True

    def _signal_stop(self, key):
        if key not in self.stopping:
            self.stopping.add(key)
            self.owned[key].stop()
            self.report(f'正在停止 {key}')

    def stop(self, key):
        if key not in self.owned:
            if self.present(key):
                self.report('该节点由外部终端启动，请在原终端停止。')
            return
        if key in ('rtk', 'sensors', 'driver') and self.nav_present and not self.stopping_all:
            self.report('请先停止导航，再关闭定位或底盘。')
            return
        if key == 'navigation' and not self.stopping_all and (
                self.calibrating or 'wgs84' in self.owned
                or self.runtime.get('busy')
                or any(task_in_progress(self.recent(mode)) for mode in ('v1', 'v2'))):
            self.report('导航任务或配准仍在运行，请先取消任务或使用一键全部停止。')
            return
        if key == 'rtk':
            self.startup_calibration = None
        self._signal_stop(key)

    def stop_all(self):
        if self.stopping_all:
            return
        self.grasp_handoff = None
        self.grasp_mux = self.grasp_unpark = None
        self.startup_calibration = None
        self.calibration_hold = self.calibration_hold_exited_at = None
        if self.idle:
            self.report('本面板没有可停止的进程。外部流程请在原终端停止。')
            return
        self.stopping_all = True
        self.report('正在停止本面板启动的流程。')
        if 'vision' in self.owned:
            self._signal_stop('vision')
        self._advance_stop()

    def _advance_stop(self):
        if 'calibration_hold' in self.owned:
            return
        for key in STOP_ORDER:
            if key in self.owned:
                self._signal_stop(key)
                return
        for key in AUX:
            if key in self.owned:
                self._signal_stop(key)
                return
        self.stopping_all = False
        self.report('本面板启动的流程已停止。')

    @property
    def idle(self):
        return not self.owned

    def _nav_idle(self):
        items = self.recent('action')
        return (not self.runtime.get('busy') and 'wgs84' not in self.owned
                and items is not None and not any(state in ACTIVE_ACTION_STATES for _, state in items)
                and not any(task_in_progress(self.recent(mode)) for mode in ('v1', 'v2')))

    def _safe_to_move(self, mux=None):
        mux = self.mux if mux is None else mux
        return (mux.get('manual_hold') in (True, False)
                and mux.get('teleop_fresh') is False
                and mux.get('emergency_stop_latched') is False)

    def _calibration_block(self):
        for key, title in (('sensors', '定位与传感器'), ('driver', '底盘驱动'), ('navigation', '导航')):
            if not self.ready(key):
                return f'请先等待{title}就绪。'
        if self.graph.frame != 'odom':
            return '直行诊断仅在 odom 导航下可用。'
        if not self._nav_idle():
            return '请先结束运行中的导航或空地任务。'
        if not self._safe_to_move() or self.mux.get('manual_hold') is not False:
            return '请在控制端核对停车来源、急停和接管状态。直行诊断需要导航已放行。'
        if self.recent('odom_still') is not True:
            return '请确认车辆停稳，并等待本地里程计。'
        if not self.geo:
            return '等待地理配准节点发布状态。'
        if self.present('calibrate'):
            return '当前已有配准进程。'
        return None

    def _roundtrip_block(self):
        if self.recent('odom') is None:
            return '等待本地里程计'
        if self.board.fixed_position(self.clock()) is None:
            return '等待 RTK Fixed'
        if self.recent('driver') is not True or not self.recent('lio'):
            return '等待底盘和 LIO 就绪'
        if self.recent('wheel_still') is not True:
            return '等待轮速确认停车'
        if (self.mux.get('selected_source') in ('mission', 'teleop') or self.grasp_handoff
                or self.grasp_state in ('starting', 'busy') or any(key in self.owned for key in AUX)):
            return '等待车辆控制链空闲'
        block = self._calibration_block()
        return block.rstrip('。') if block else None

    def _await_navigation_map(self):
        self.startup_calibration = 'awaiting_map'
        self.map_handoff_deadline = self.clock() + MAP_HANDOFF_TIMEOUT

    def _fail_map_handoff(self, reason):
        self.startup_calibration = 'map_failed'
        self.map_handoff_failure = '切图失败 · ' + reason
        self.report('配准已完成 · ' + self.map_handoff_failure)

    def _advance_startup_calibration(self):
        state = self.startup_calibration
        if 'rtk' not in self.owned and not self.present('rtk'):
            self.startup_calibration = None
        elif 'navigation' not in self.owned or 'navigation' in self.stopping:
            return
        elif state in ('waiting', 'awaiting_map', 'failed', 'map_failed') and self.ready('map') and not self.switching:
            self.startup_calibration = None
        elif state == 'waiting':
            if self._roundtrip_block() is None:
                if self.ready('calibrate'):
                    self._await_navigation_map()
                elif self.odom_cruise and self.runtime.get('preparation') is not True:
                    if self._start('prepare', SWITCH_PREPARATION):
                        self.startup_calibration = 'preparing'
                        self.prepare_started = self.clock()
                    else:
                        self.startup_calibration = 'failed'
                elif self._start('calibrate', ROUNDTRIP):
                    self.startup_calibration = 'running'
                    self.calibrating = True
                else:
                    self.startup_calibration = 'failed'
        elif state == 'preparing':
            if self.runtime.get('phase') == 'BLOCKED' or self.clock() - self.prepare_started > 30.:
                self.startup_calibration = 'failed'
            elif self.runtime.get('preparation') is True and not self.switching:
                self.odom_cruise = False
                self.startup_calibration = 'waiting'
        elif state == 'awaiting_map':
            block = self._map_block()
            if self.clock() >= self.map_handoff_deadline:
                self._fail_map_handoff(block or '等待地图交接超时')
            elif block is None:
                if self._map():
                    self.startup_calibration = 'switching'
                else:
                    self._fail_map_handoff('未能启动地图交接')
        elif state == 'switching':
            if self.runtime.get('phase') == 'BLOCKED':
                self._fail_map_handoff(self.runtime.get('reason') or '导航运行器拒绝切换')
            elif self.ready('map') and not self.switching:
                self.startup_calibration = None

    def _move_control(self, key, action, *options):
        if '/ugv/control' in self.graph.nodes and not any(item in self.owned for item in AUX):
            self.report('控制台已在操作停车状态，请稍后再试。')
            return False
        command = ((sys.executable, str(CONTROL_SOURCE))
                   if key in ('grasp_hold', 'calibration_hold') else CONTROL)
        return self._start(key, (*command, action, '--timeout', '4', *options))

    def _wgs84_block(self, position):
        if position is None:
            return '等待 RTK Fixed'
        if self.recent('driver') is not True or not self.ready('driver'):
            return '等待底盘就绪'
        if self.recent('odom') is None:
            return '等待本地里程计'
        mux = self.mux
        if (not self._safe_to_move(mux) or mux.get('manual_hold') is not False
                or mux.get('selected_source') == 'mission'):
            return '等待 Mux 放行'
        if (self.stopping_all or self.calibrating or self.grasp_handoff or self.switching
                or self.startup_calibration == 'awaiting_map'
                or any(key in self.owned for key in AUX)
                or (self.present('vision') and 'vision' not in self.owned)
                or self.grasp_state in ('starting', 'busy')):
            return '等待当前流程结束'
        connected = self._navigation_connected() if self.runtime else self.ready('map')
        if not connected or not self._nav_idle():
            return '等待导航空闲'
        return None

    def can_send_wgs84(self):
        return self._wgs84_block(self.board.fixed_position(self.clock())) is None

    def send_wgs84(self, latitude, longitude):
        position = self.board.fixed_position(self.clock())
        blocked = self._wgs84_block(position)
        if blocked:
            self.report(blocked)
            return False
        try:
            latitude, longitude = map(float, (latitude, longitude))
        except (TypeError, ValueError, OverflowError):
            self.report('请填写数值经纬度。')
            return False
        if not (-90. <= latitude <= 90. and -180. <= longitude <= 180.):
            self.report('请检查 WGS84 经纬度范围。')
            return False
        altitude = position['altitude']
        command = ('rosrun', 'ugv_navigation', 'geographic.py', 'send',
                   '--latitude', str(latitude), '--longitude', str(longitude),
                   '--altitude-ellipsoid', str(altitude))
        self.board.record('wgs84_failure', None, self.clock())
        self.board.record('wgs84', {'latitude': latitude, 'longitude': longitude}, self.clock())
        if self._start('wgs84', command):
            return True
        self.board.record('wgs84', None, self.clock())
        return False

    def _base_click(self):
        groups = ('sensors', 'driver')
        if not self.graph.online or any(key in self.stopping for key in groups):
            self.report('等待 ROS Master 和硬件进程状态。')
            return
        missing = [key for key in groups if key not in self.owned and not self.present(key)]
        if missing:
            for key in missing:
                self.click(key)
        elif any(key in self.owned for key in groups):
            if self.nav_present:
                self.report('请先停止导航，再关闭雷达或底盘。')
                return
            for key in reversed(groups):
                if key in self.owned:
                    self.stop(key)
        else:
            self.report('雷达和底盘由外部终端启动。')

    def _rtk_click(self):
        if not self.graph.online:
            self.report('等待全局 ROS Master。')
            return
        if self.startup_calibration_pending:
            self.report('当前已在配准或切换导航。')
            return
        if 'rtk' in self.owned:
            self.stop('rtk')
            return
        if not self.present('rtk'):
            home = Path(self.env.get('HOME', str(Path.home())))
            gnss = self.env.get('SLICER_GNSS_CONFIG') or str(home / '.config/slicer/gnss.yaml')
            if not self._start('rtk', ('roslaunch', 'ugv_bringup', 'slicer_field_sensors.launch',
                                       'start_local_sensors:=false', f'config_file:={gnss}')):
                return
        self.startup_calibration = 'waiting'
        self.report('RTK 已登记自动配准，定位、底盘与导航就绪后将前进约 2 m 并倒车回起点。')

    def click(self, key, mode='v1', confirmed=False, process_groups=None):
        if self.stopping_all or key in self.stopping:
            return
        if process_groups is not None:
            self.graph.processes = process_groups
        if key == 'rtk':
            return self._rtk_click()
        if key == 'base':
            return self._base_click()
        if key == 'vision' and self.grasp_handoff is not None:
            return
        if key in self.owned and key != 'calibrate':
            if key == 'vision':
                if self.grasp_state not in ('starting', 'busy'):
                    self._request_grasp()
            else:
                self.stop(key)
            return
        if self.present(key) and key not in ('calibrate', 'map'):
            pending = '' if self.ready(key) else self._waiting_status(key) + '。'
            self.report(pending + '该步骤由外部终端启动。需要重启时请先在原终端停止。')
            return
        if not self.graph.online and key != 'vision':
            self.report('等待全局 ROS Master。')
            return
        if key in ('driver', 'navigation', 'map', 'calibrate') and (
                self.grasp_handoff or (self.present('vision') and 'vision' not in self.owned)
                or ('vision' in self.owned and self.grasp_state in ('starting', 'busy'))):
            self.report('等待视觉抓取结束并交还底盘串口。')
            return
        if key == 'vision':
            self._request_grasp()
            return
        if key == 'calibrate':
            return self._calibrate(confirmed)
        if key == 'map':
            self._map()
            return
        if key == 'sensors' and (self.graph.publishers.get('/scan') or self.graph.publishers.get('/lio/odom')):
            self.report('已有雷达或 LIO 发布源，请先停止原定位入口。')
            return
        if key == 'driver' and self.graph.publishers.get('/wheel/odom'):
            self.report('已有轮速里程计发布源，请先检查底盘驱动。')
            return
        commands = {
            'sensors': ('roslaunch', 'ugv_bringup', 'slicer_field_sensors.launch', 'geographic:=false'),
            'driver': DRIVER_COMMAND,
            'navigation': NAVIGATION,
            'air': ('roslaunch', 'ugv_bringup', 'slicer_air_ground.launch'),
        }
        if key == 'air' and mode == 'v2':
            ip = self.env.get('SLICER_UAV_IP')
            commands[key] = ('roslaunch', 'ag_ugv_task', 'ugv.launch', 'monitor_only:=false',
                             f'start_fkie:={str(bool(ip)).lower()}')
            if ip:
                commands[key] += (f'uav_ip:={ip}',)
        if key == 'navigation':
            cruise = (self.startup_calibration != 'waiting'
                      or self.board.fixed_position(self.clock()) is None)
            if cruise:
                commands[key] += ('odom_cruise:=true',)
            if self._start(key, commands[key]):
                self.odom_cruise = cruise
        else:
            self._start(key, commands[key])

    def _calibrate(self, confirmed):
        if self.calibrating or self.ready('calibrate') or self.present('calibrate'):
            self.report('当前配准已完成或正在执行。')
            return
        if not self.board.view(self.clock())['usable']:
            self.report('请等待新鲜 RTK FIXED（gga_quality=4）。')
            return
        block = self._calibration_block()
        if block:
            self.report(block)
            return
        if any(key in self.owned for key in AUX):
            self.report('请先等待停车状态操作结束。')
            return
        if not confirmed:
            return 'confirm'
        self.calibrating = self._start(
            'calibrate', ('rosrun', 'ugv_navigation', 'geographic.py', 'initialize', '--distance', '2.5'))
        if self.calibrating:
            self.startup_calibration = None

    def _end_registration(self):
        self.calibration_hold = self.calibration_hold_exited_at = None
        instance, generation = self._mux_key(self.mux)
        if (self.graph.online and self.mux.get('manual_hold') is False
                and instance and type(generation) is int and self._safe_to_move()
                and self._move_control('calibration_hold', 'hold', '--require-idle',
                                       '--expected-mux-instance', instance,
                                       '--expected-generation', str(generation))):
            return
        self.calibrating = False
        self.report('请查看 Mux，确认诊断结束后车辆保持停车。')

    def _map_block(self):
        if self.switching or self.graph.frame == 'map':
            return ''
        if not self.ready('calibrate'):
            return '请等待地理配准就绪。'
        if not (self.ready('sensors') and self.ready('driver')):
            return '切换前请等待定位与底盘就绪。'
        if self.nav_present and 'navigation' not in self.owned:
            return '导航由外部终端启动，请在该终端管理，或由统一入口提交 WGS84 任务。'
        if self.recent('odom_still') is not True or (
                'navigation' in self.owned and not (self._safe_to_move() and self._nav_idle())):
            return '切换导航前请结束任务，确认车辆停稳。'
        if any(key in self.owned for key in AUX) or self.calibrating:
            return '请等待配准与停车操作结束。'
        if self.calibration_hold_exited_at is not None and (
                self.graph.started_at is None or self.graph.started_at <= self.calibration_hold_exited_at):
            return '等待停车进程退出后发起的 ROS 节点检查。'
        if 'navigation' in self.owned:
            if self.calibration_hold is not None and (
                    self.calibration_hold != self._mux_key(self.mux)
                    or self.mux.get('manual_hold') is not True):
                return '配准停车后的 Mux 状态已变化，请核对接管状态。'
            if '/ugv/control' in self.graph.nodes:
                return '等待外部停车控制台退出。'
            if self.mux.get('manual_hold') is not False and self.calibration_hold is None:
                return '请在原控制端核对并解除其停车状态。'
        return None

    def _map(self):
        block = self._map_block()
        if block is not None:
            if block:
                self.report(block)
            return False
        if 'navigation' not in self.owned:
            return self._start('navigation', (*NAVIGATION, 'initial_mode:=map'))
        if self.mux.get('manual_hold') is False:
            return self._start('map', SWITCH_MAP)
        instance, generation = self.calibration_hold
        return self._move_control('map_release', 'release',
                                  '--expected-mux-instance', instance, '--expected-generation', str(generation))

    def _exit(self, key, status, log):
        stopping = key in self.stopping
        self.stopping.discard(key)
        if not stopping and key in STEPS and (key != 'calibrate' or status != 0):
            if key != 'vision' or self.grasp_state != 'error':
                self.exited[key] = '进程已退出' if status == 0 else '运行失败'
            self.report(f'{key} 已退出（状态 {status}）' + (f' · {log}' if log else ''))
        elif status == 0:
            self.report(f'{key} 已退出')
        elif not stopping:
            self.report(f'{key} 退出（状态 {status}），请查看 ROS 日志。')
        if key == 'driver':
            for sample in ('driver', 'wheel_odom', 'wheel_still'):
                self.board.record(sample, None, self.clock())
        if key == 'vision':
            if self.grasp_state != 'starting':
                self.grasp_recovery = self.grasp_state in ('busy', 'held', 'recovery')
            self.grasp_command = None
            if self.grasp_handoff == 'grasp':
                self._set_grasp_handoff('to_driver')
        if key == 'grasp_hold' and self.grasp_handoff == 'parking':
            if status == 0 and self.grasp_mux is not None:
                self.grasp_unpark = self.grasp_mux
                self._set_grasp_handoff('parked')
            else:
                self.grasp_handoff = None
                self.report('未确认导航停车，抓取未启动。')
        if key == 'grasp_unhold':
            self.grasp_handoff = None
            self.grasp_mux = self.grasp_unpark = None
            if status != 0 and not self.stopping_all:
                self.report('导航仍保持停车，请检查 Mux。')
        if key == 'wgs84':
            if status != 0 and not stopping:
                self.board.record('wgs84_failure', self.board.local_target(), self.clock())
            self.board.record('wgs84', None, self.clock())
        if key == 'calibrate':
            if self.startup_calibration == 'running':
                if status == 0 and not stopping:
                    self._await_navigation_map()
                else:
                    self.startup_calibration = 'failed'
            self._end_registration()
        elif key == 'calibration_hold':
            if status != 0 or stopping or self.stopping_all:
                self.calibration_hold = None
            if self.startup_calibration == 'awaiting_map' and self.calibration_hold is None:
                self._fail_map_handoff('未确认配准停车回执')
            self.calibration_hold_exited_at = None if self.stopping_all else self.clock()
            self.calibrating = False
        elif key == 'map_release':
            self.calibration_hold = None
            switched = (status == 0 and not self.stopping_all and 'navigation' in self.owned
                        and self._start('map', SWITCH_MAP))
            if not switched and self.startup_calibration == 'switching':
                self._fail_map_handoff('配准停车交接或地图切换请求失败')
        elif key == 'map' and status != 0 and self.startup_calibration == 'switching':
            self._fail_map_handoff('地图切换服务调用失败')
        elif key == 'prepare' and status != 0 and self.startup_calibration == 'preparing':
            self.startup_calibration = 'failed'
        elif key == 'navigation':
            self.odom_cruise = False
            if self.startup_calibration in ('awaiting_map', 'switching'):
                self._fail_map_handoff('导航运行器已退出')
            elif self.startup_calibration in ('preparing', 'running'):
                self.startup_calibration = 'failed'

    def update(self, graph=None, read_logs=True):
        if graph is not None:
            self.graph = graph
        for key, proc in list(self.owned.items()):
            if key in AUX and self.clock() - self.started[key] > 12. and key not in self.stopping:
                self.report(f'{key} 操作超时，请检查 ROS Master 和 Mux。')
                self._signal_stop(key)
            status = proc.poll()
            if read_logs or status is not None:
                lines = proc.lines()
                if key == 'vision':
                    self._grasp_log(lines, status is None)
                elif key in ('grasp_hold', 'calibration_hold'):
                    for line in lines:
                        if line.startswith('{'):
                            try:
                                held = json.loads(line)
                            except ValueError:
                                continue
                            if (held.get('last_intent_action') == 'takeover' and held.get('manual_hold') is True
                                    and held.get('last_intent_accepted') is True):
                                if key == 'calibration_hold':
                                    instance, generation = self._mux_key(held)
                                    if instance and type(generation) is int:
                                        self.calibration_hold = instance, generation
                                elif self.grasp_handoff == 'parking':
                                    self.grasp_mux = self._mux_key(held)
                if 'success: False' in lines:
                    if key == 'map' and self.startup_calibration == 'switching':
                        reason = next((line.partition(':')[2].strip() for line in lines
                                       if line.startswith('message:')), '')
                        self._fail_map_handoff(reason or '导航运行器拒绝切换')
                    elif key == 'prepare' and self.startup_calibration == 'preparing':
                        self.startup_calibration = 'failed'
                concise = [line for line in lines if any(
                    token in line for token in ('ERROR', 'FATAL', 'RLException', 'PermissionError',
                                                'Traceback', '相机', '[READY]', 'control:', '"reason":', 'message:'))]
                if concise:
                    self.report(concise[-1][:210])
            if status is not None:
                del self.owned[key]
                del self.started[key]
                self._exit(key, status, getattr(proc, 'log', None))
        if self.stopping_all:
            self._advance_stop()
        elif self.grasp_handoff is not None:
            self._advance_grasp_handoff(fresh_graph=graph is not None)
        elif self.startup_calibration is not None:
            self._advance_startup_calibration()
