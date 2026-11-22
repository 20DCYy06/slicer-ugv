"""Orchestrate the existing launch files on the user's ROS master."""
from collections import deque
import json
import os
from pathlib import Path
import shlex
import signal
import subprocess
import sys
import tempfile
import time

from dashboard_model import ACTIVE_ACTION_STATES, task_in_progress
from workflow_discovery import Graph, NAV_ACTION

STEPS = ('sensors', 'driver', 'vision', 'odom', 'calibrate', 'map', 'air')
STOP_ORDER = ('grasp_hold', 'grasp_unhold', 'startup_release', 'release', 'calibrate', 'map', 'odom',
              'vision', 'air', 'driver', 'sensors')
NODE_GROUPS = {
    'sensors': {'/geographic', '/cup_gnss', '/slicer_cloud_to_scan', '/slicer_ekf'},
    'driver': {'/slicer_c30d_driver'},
}
AUX = ('startup_release', 'release', 'switch_hold', 'calibration_hold', 'stop_nav_hold',
       'grasp_hold', 'grasp_unhold')
CONTROL = ('rosrun', 'ugv_slicer_control', 'control.py')
SERIAL_GROUP = Path(__file__).resolve().parents[2] / 'manual' / 'serial_group.sh'
FIELD_NAVIGATION = (Path(__file__).resolve().parents[3] / 'src' / 'ugv_bringup' /
                    'launch' / 'slicer_field_navigation.launch')
CONTROL_SOURCE = FIELD_NAVIGATION.parents[2] / 'ugv_slicer_control/scripts/control.py'
VISUAL_GRASP = ('rosrun', 'slicer_visual_grasp', 'run.sh', '--run', '--headless')
DRIVER_COMMAND = ('roslaunch', 'ugv_bringup', 'slicer_body_driver.launch')
# The process runner escalates shutdown at 32 s. Allow another ROS graph poll.
HANDOFF_TIMEOUT = 35.
AIR_PEER_TOPICS = {'v1': '/uav/test', 'v2': '/air_ground/v2/uav_state'}
GRASP_STATES = {'idle', 'busy', 'held', 'recovery', 'error'}


class OwnedProcess:
    def __init__(self, key, argv, env):
        if key in ('sensors', 'driver'):
            # The setuid sg helper strips the loader path; restore it for the ROS binaries.
            if 'LD_LIBRARY_PATH' in env:
                argv = ('env', f'LD_LIBRARY_PATH={env["LD_LIBRARY_PATH"]}', *argv)
            argv = ('bash', '-c', 'source "$1" && shift && run_serial_tool "$@"',
                    'rtk-desktop', str(SERIAL_GROUP), *argv)
        log_dir = Path(env.get('ROS_HOME') or Path.home() / '.ros') / 'log'
        log_dir.mkdir(parents=True, exist_ok=True)
        with tempfile.NamedTemporaryFile(prefix=f'rtk-desktop-{key}-', suffix='.log',
                                         dir=str(log_dir), delete=False) as stream:
            self.log = Path(stream.name)
            self.process = subprocess.Popen(argv, env=env,
                                            stdin=subprocess.PIPE if key == 'vision' else subprocess.DEVNULL,
                                            stdout=stream, stderr=subprocess.STDOUT,
                                            start_new_session=True)
        self.offset = 0
        self.pending_line = b''
        self.stop_at = None
        self.signal_stage = 0

    def send(self, command):
        self.process.stdin.write((command + '\n').encode('ascii'))
        self.process.stdin.flush()

    def _signal(self, sig):
        try:
            os.killpg(self.process.pid, sig)
        except ProcessLookupError:
            return False
        return True

    def stop(self):
        if self.stop_at is None:
            self.stop_at = time.monotonic()
            self._signal(signal.SIGINT)

    def poll(self):
        status = self.process.poll()
        if self.stop_at is None:
            return status
        # sg may exit on SIGTERM before its child finishes closing the serial port.
        if status is not None and (self.signal_stage == 2 or not self._signal(0)):
            return status
        elapsed = time.monotonic() - self.stop_at
        # Allow roslaunch and Tk to finish their own shutdown handlers.
        if elapsed > 32. and self.signal_stage == 1:
            self.signal_stage = 2
            self._signal(signal.SIGKILL)
        elif elapsed > 25. and self.signal_stage == 0:
            self.signal_stage = 1
            self._signal(signal.SIGTERM)
        return None

    def lines(self):
        try:
            with self.log.open('rb') as stream:
                # Skip old verbose output rather than stalling the GUI.
                stream.seek(max(self.offset, self.log.stat().st_size - 8192))
                data = self.pending_line + stream.read(8192)
                self.offset = stream.tell()
        except OSError:
            return []
        *lines, self.pending_line = data.split(b'\n')
        if self.process.poll() is not None and self.pending_line:
            lines.append(self.pending_line)
            self.pending_line = b''
        return [line.decode(errors='replace').strip() for line in lines if line.strip()]


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
        self.switching = False
        self.switch_deadline = 0.
        self.nav_to_stop = None
        self.calibrating = False
        self.calibration_started = False
        self.stopping_all = False
        self.next_startup_release = 0.
        self.startup_release_key = None
        self.release_interrupted = False

    def recent(self, key):
        return self.board.recent(key, self.clock(), 2.5)

    def report(self, message):
        self.messages.append(message)
        print(f'RTK 流程 · {message}', flush=True)

    def present(self, key):
        graph = self.graph
        if key == 'air':
            return bool(graph.air_mode or graph.processes.get('air'))
        if key in ('odom', 'map'):
            return bool(graph.processes.get(key) or
                        (graph.frame == key and NAV_ACTION in graph.nodes))
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
    def nav_present(self):
        return bool('odom' in self.owned or 'map' in self.owned
                    or self.present('odom') or self.present('map') or self.graph.processes.get('nav')
                    or {NAV_ACTION, '/slicer_nav_arc_guard', '/slicer_velocity_mux'} & self.graph.nodes
                    or self.graph.publishers.get('/cmd_vel'))

    @property
    def vision_driver_busy(self):
        return ('driver' in self.owned or self.present('driver') or self.ready('driver')
                or bool(self.graph.publishers.get('/wheel/odom')))

    @property
    def air_local_modes(self):
        return tuple(mode for mode in (self.graph.air_mode or '').split('+')
                     if mode and self.recent(mode) is not None)

    def ready(self, key):
        if key == 'vision':
            return self.grasp_state == 'held' and 'vision' in self.owned
        if not self.graph.online:
            return False
        if key == 'sensors':
            return ((bool(self.owned.get(key)) or self.present(key))
                    and bool(self.recent('lidar') and
                             (self.recent('odom') is not None or self.recent('gnss'))))
        if key == 'driver':
            ready = self.recent('driver')
            return (bool(self.owned.get(key)) or self.present(key)) and (
                ready is True or (ready is None and self.recent('wheel_odom') is True))
        if key in ('odom', 'map'):
            return (self.graph.frame == key and self.recent('action') is not None
                    and NAV_ACTION in self.graph.subscribers.get(NAV_ACTION + '/goal', ())
                    and (key == 'odom' or self.ready('calibrate')))
        if key == 'calibrate':
            return self.geo.get('state') == 'ready' and self.geo.get('registered') is True
        if key == 'air':
            return any((mode == 'v1' and self.recent('aircraft_v1_command') is True)
                       or self.recent('aircraft_' + mode) in
                       self.graph.publishers.get(AIR_PEER_TOPICS[mode], ())
                       for mode in self.air_local_modes)

    def _waiting_status(self, key):
        if key == 'sensors':
            return '等待雷达扫描' if not self.recent('lidar') else '等待定位数据'
        if key == 'driver':
            return '等待底盘遥测'
        if key in ('odom', 'map') and self.recent('odom') is None:
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
            if key in self.stopping or key == self.nav_to_stop:
                result[key] = ('yellow', '停止中', False)
            elif key == 'map' and self.switching:
                result[key] = ('yellow', '切换中', False)
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
                result[key] = ('green', status, key != 'calibrate' and not self.stopping_all)
            elif owned or external:
                result[key] = ('yellow', self._waiting_status(key), not self.stopping_all)
            else:
                result[key] = ('red', self.exited.get(key, ''), not self.stopping_all)
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
        if self.switching or self.calibrating or any(key in self.owned for key in AUX):
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
            if key == 'startup_release' and (self.stopping_all or self.nav_to_stop):
                self.release_interrupted = True
            self.stopping.add(key)
            self.owned[key].stop()
            self.report(f'正在停止 {key}')

    def stop(self, key):
        if key not in self.owned:
            if self.present(key):
                self.report('该节点由外部终端启动，请在原终端停止。')
            return
        if key in ('sensors', 'driver') and self.nav_present and not self.stopping_all:
            self.report('请先停止导航，再关闭定位或底盘。')
            return
        if key in ('odom', 'map'):
            self._stop_navigation(key)
            return
        self._signal_stop(key)

    def _stop_navigation(self, key):
        if self.nav_to_stop is not None or key in self.stopping:
            return
        if self.calibrating and not self.stopping_all:
            self.report('配准中，请使用一键全部停止。')
            return
        if 'startup_release' in self.owned:
            self.nav_to_stop = key
            self._signal_stop('startup_release')
            return
        mux = self.mux
        key_matches = (mux.get('mux_instance_id'), mux.get('takeover_generation')) == self.startup_release_key
        # A release just sent by control.py may have reached Mux ahead of the last observed state.
        need_hold = (mux.get('manual_hold') is False or self.release_interrupted
                     or (key_matches and mux.get('hold_reason') == 'startup'))
        if (self.graph.online and need_hold
                and not any(item in self.owned for item in AUX)
                and self._move_control('stop_nav_hold', 'hold')):
            self.nav_to_stop = key
        else:
            self._signal_stop(key)

    def stop_all(self):
        if self.stopping_all:
            return
        self.grasp_handoff = None
        self.grasp_mux = self.grasp_unpark = None
        if self.idle and not self.switching:
            self.report('本面板没有可停止的进程。外部流程请在原终端停止。')
            return
        self.switching = False
        self.stopping_all = True
        self.report('正在停止本面板启动的流程。')
        if 'vision' in self.owned:
            self._signal_stop('vision')
        self._advance_stop()

    def _advance_stop(self):
        if 'calibration_hold' in self.owned or 'stop_nav_hold' in self.owned:
            return
        for key in STOP_ORDER:
            if key in self.owned:
                if key in ('odom', 'map'):
                    self._stop_navigation(key)
                else:
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
        return (items is not None and not any(state in ACTIVE_ACTION_STATES for _, state in items)
                and not any(task_in_progress(self.recent(mode)) for mode in ('v1', 'v2')))

    def _safe_to_move(self):
        return (self.mux.get('manual_hold') in (True, False)
                and self.mux.get('teleop_fresh') is False
                and self.mux.get('emergency_stop_latched') is False)

    def _calibration_block(self):
        for key, title in (('sensors', '定位与传感器'), ('driver', '底盘驱动'), ('odom', '起步导航 (odom)')):
            if not self.ready(key):
                return f'请先等待{title}就绪。'
        if not self._nav_idle():
            return '请先结束运行中的导航或空地任务。'
        if not self._safe_to_move():
            return '请检查 Mux 状态、急停与手动控制。'
        if self.recent('odom_still') is not True:
            return '请确认车辆停稳，并等待本地里程计。'
        if not self.geo:
            return '等待地理配准节点发布状态。'
        if self.present('calibrate'):
            return '当前已有配准进程。'
        return None

    def _move_control(self, key, action, *options):
        if '/ugv/control' in self.graph.nodes and not any(item in self.owned for item in AUX):
            self.report('控制台已在操作停车状态，请稍后再试。')
            return False
        command = (sys.executable, str(CONTROL_SOURCE)) if key == 'grasp_hold' else CONTROL
        return self._start(key, (*command, action, '--timeout', '4', *options))

    def _release_startup_hold(self):
        if (self.calibrating or self.nav_to_stop or self.stopping
                or any(key in self.owned for key in AUX) or self.clock() < self.next_startup_release):
            return
        if not any(key in self.owned and self.ready(key) for key in ('odom', 'map')):
            return
        mux = self.mux
        instance, generation = mux.get('mux_instance_id'), mux.get('takeover_generation')
        if (mux.get('manual_hold') is not True or mux.get('hold_reason') != 'startup'
                or not isinstance(instance, str) or not instance or type(generation) is not int
                or not (self._safe_to_move() and self._nav_idle() and self.ready('driver'))
                or self.board.recent('driver', self.clock(), 1.5) is not True):
            return
        self.next_startup_release = self.clock() + 2.
        if self._move_control('startup_release', 'release', '--expected-mux-instance', instance,
                              '--expected-generation', str(generation)):
            self.startup_release_key = instance, generation

    def click(self, key, mode='v1', confirmed=False, process_groups=None):
        if self.stopping_all or key in self.stopping:
            return
        if process_groups is not None:
            self.graph.processes = process_groups
        if key == 'vision' and self.grasp_handoff is not None:
            return
        if key in self.owned and key != 'calibrate':
            if key == 'vision':
                if self.grasp_state not in ('starting', 'busy'):
                    self._request_grasp()
            else:
                self.stop(key)
            return
        if self.present(key) and key != 'calibrate':
            pending = '' if self.ready(key) else self._waiting_status(key) + '。'
            self.report(pending + '该步骤由外部终端启动。需要重启时请先在原终端停止。')
            return
        if not self.graph.online and key != 'vision':
            self.report('等待全局 ROS Master。')
            return
        if key in ('driver', 'odom', 'map', 'calibrate') and (
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
        if key == 'odom' and (self.nav_present or self.switching or 'map' in self.owned):
            self.report('已有导航或速度 Mux，请先停止。')
            return
        if key == 'sensors' and (self.graph.publishers.get('/scan') or self.graph.publishers.get('/lio/odom')):
            self.report('已有雷达或 LIO 发布源，请先停止原定位入口。')
            return
        if key == 'driver' and self.graph.publishers.get('/wheel/odom'):
            self.report('已有轮速里程计发布源，请先检查底盘驱动。')
            return
        home = Path(self.env.get('HOME', str(Path.home())))
        gnss = self.env.get('SLICER_GNSS_CONFIG') or str(home / '.config/slicer/gnss.yaml')
        commands = {
            'sensors': ('roslaunch', 'ugv_bringup', 'slicer_field_sensors.launch', f'config_file:={gnss}'),
            'driver': DRIVER_COMMAND,
            'odom': ('roslaunch', str(FIELD_NAVIGATION), 'geographic:=false'),
            'air': ('roslaunch', 'ugv_bringup', 'slicer_air_ground.launch'),
        }
        if key == 'air' and mode == 'v2':
            ip = self.env.get('SLICER_UAV_IP')
            commands[key] = ('roslaunch', 'ag_ugv_task', 'ugv.launch', 'monitor_only:=false',
                             f'start_fkie:={str(bool(ip)).lower()}')
            if ip:
                commands[key] += (f'uav_ip:={ip}',)
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
        self.calibrating = True
        if self.mux.get('manual_hold') is True:
            self.calibrating = self._move_control('release', 'release')
        else:
            self._start_registration()

    def _start_registration(self):
        if (self._calibration_block() is None and self.mux.get('manual_hold') is False
                and self.board.view(self.clock())['usable']):
            self.calibration_started = self._start(
                'calibrate', ('rosrun', 'ugv_navigation', 'geographic.py', 'initialize', '--distance', '2.5'))
        if not self.calibration_started:
            self.report('配准启动前状态变化，保持停车。')
            self._end_registration()

    def _end_registration(self):
        self.calibration_started = False
        if self.graph.online and self.mux:
            if self._move_control('calibration_hold', 'hold'):
                return
        self.calibrating = False
        self.report('未确认停车状态，请查看 Mux。')

    def _map(self):
        if self.switching:
            return
        if not self.ready('calibrate'):
            self.report('请先完成第 5 步地理配准。')
            return
        if not (self.ready('sensors') and self.ready('driver')):
            self.report('切换前请等待定位与底盘就绪。')
            return
        if self.nav_present and 'odom' not in self.owned:
            self.report('odom 导航由外部终端启动。请在原终端停掉导航，保留定位和底盘。')
            return
        if self.recent('odom_still') is not True or (
                'odom' in self.owned and not (self._safe_to_move() and self._nav_idle())):
            self.report('切换导航前请结束任务，确认车辆停稳。')
            return
        if any(key in self.owned for key in AUX) or self.calibrating:
            self.report('请等待配准与停车操作结束。')
            return
        self.switching = True
        self.switch_deadline = self.clock() + 30.
        if 'odom' in self.owned and self.mux.get('manual_hold') is False:
            self.switching = self._move_control('switch_hold', 'hold')
        elif 'odom' in self.owned:
            self._signal_stop('odom')

    def _advance_map(self):
        if self.clock() > self.switch_deadline:
            self.switching = False
            self.report('等待旧导航退出超时。')
        elif 'odom' not in self.owned and 'switch_hold' not in self.owned and not self.nav_present:
            self.switching = False
            if self.ready('calibrate') and self.ready('sensors') and self.ready('driver'):
                self._start('map', ('roslaunch', str(FIELD_NAVIGATION)))
            else:
                self.report('定位、底盘或配准状态变化，暂停切换。')

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
        if key in ('odom', 'map'):
            self.startup_release_key = None
            self.release_interrupted = False
        if key == 'startup_release' and self.nav_to_stop is not None:
            target, self.nav_to_stop = self.nav_to_stop, None
            if target in self.owned:
                self._stop_navigation(target)
        if key == 'calibrate':
            self._end_registration()
        elif key == 'release' and self.calibrating:
            if status == 0 and not self.stopping_all:
                self._start_registration()
            else:
                self._end_registration()
        elif key == 'calibration_hold':
            self.calibrating = False
        elif key == 'switch_hold' and self.switching:
            if status == 0 and 'odom' in self.owned:
                self._signal_stop('odom')
            else:
                self.switching = False
        elif key == 'stop_nav_hold':
            target, self.nav_to_stop = self.nav_to_stop, None
            if target in self.owned:
                self._signal_stop(target)

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
                elif key == 'grasp_hold' and self.grasp_handoff == 'parking':
                    for line in lines:
                        if line.startswith('{'):
                            try:
                                held = json.loads(line)
                            except ValueError:
                                continue
                            if (held.get('last_intent_action') == 'takeover' and held.get('manual_hold') is True
                                    and held.get('last_intent_accepted') is True):
                                self.grasp_mux = self._mux_key(held)
                concise = [line for line in lines if any(
                    token in line for token in ('ERROR', 'FATAL', 'RLException', 'PermissionError',
                                                'Traceback', '相机', '[READY]', 'control:', '"reason":'))]
                if concise:
                    self.report(concise[-1][:210])
            if status is not None:
                del self.owned[key]
                del self.started[key]
                self._exit(key, status, getattr(proc, 'log', None))
        if self.stopping_all:
            self._advance_stop()
        elif self.switching:
            self._advance_map()
        elif self.grasp_handoff is not None:
            self._advance_grasp_handoff(fresh_graph=graph is not None)
        else:
            self._release_startup_hold()
