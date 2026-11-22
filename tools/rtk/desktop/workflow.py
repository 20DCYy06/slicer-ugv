"""Orchestrate the existing launch files on the user's ROS master."""
from collections import deque
import os
from pathlib import Path
import shlex
import signal
import subprocess
import tempfile
import time

from dashboard_model import ACTIVE_ACTION_STATES, task_in_progress
from workflow_discovery import Graph, NAV_ACTION

STEPS = ('sensors', 'driver', 'odom', 'calibrate', 'map', 'vision', 'air')
STOP_ORDER = ('release', 'calibrate', 'map', 'odom', 'vision', 'air', 'driver', 'sensors')
NODE_GROUPS = {
    'sensors': {'/geographic', '/cup_gnss', '/slicer_cloud_to_scan', '/slicer_ekf'},
    'driver': {'/slicer_c30d_driver'},
}
AUX = ('release', 'hold', 'switch_hold', 'calibration_hold', 'stop_nav_hold')
CONTROL = ('rosrun', 'ugv_slicer_control', 'control.py')


class OwnedProcess:
    def __init__(self, key, argv, env):
        log_dir = Path(env.get('ROS_HOME') or Path.home() / '.ros') / 'log'
        log_dir.mkdir(parents=True, exist_ok=True)
        with tempfile.NamedTemporaryFile(prefix=f'rtk-desktop-{key}-', suffix='.log',
                                         dir=str(log_dir), delete=False) as stream:
            self.log = Path(stream.name)
            self.process = subprocess.Popen(argv, env=env, stdin=subprocess.DEVNULL,
                                            stdout=stream, stderr=subprocess.STDOUT,
                                            start_new_session=True)
        self.offset = 0
        self.pending_line = b''
        self.stop_at = None
        self.signal_stage = 0

    def _signal(self, sig):
        try:
            os.killpg(self.process.pid, sig)
        except ProcessLookupError:
            pass

    def stop(self):
        if self.stop_at is None and self.process.poll() is None:
            self.stop_at = time.monotonic()
            self._signal(signal.SIGINT)

    def poll(self):
        status = self.process.poll()
        if status is None and self.stop_at is not None:
            elapsed = time.monotonic() - self.stop_at
            # Allow roslaunch and Tk to finish their own shutdown handlers.
            if elapsed > 32. and self.signal_stage == 1:
                self.signal_stage = 2
                self._signal(signal.SIGKILL)
            elif elapsed > 25. and self.signal_stage == 0:
                self.signal_stage = 1
                self._signal(signal.SIGTERM)
        return status

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
    def __init__(self, board, spawn=OwnedProcess, env=None, clock=time.monotonic, local_scan=None):
        self.board = board
        self.spawn = spawn
        self.env = dict(os.environ if env is None else env)
        # The RViz input namespace stays private. The launched stack uses its existing names.
        self.env.pop('ROS_NAMESPACE', None)
        if self.env.get('DISPLAY'):
            self.env['VISUAL_GRASP_DISPLAY'] = self.env['DISPLAY']
        self.clock = clock
        self.local_scan = local_scan
        self.graph = Graph()
        self.vision = False
        self.owned = {}
        self.started = {}
        self.stopping = set()
        self.messages = deque(maxlen=3)
        self.switching = False
        self.switch_deadline = 0.
        self.nav_to_stop = None
        self.calibrating = False
        self.calibration_started = False
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
        if key in ('odom', 'map'):
            return bool(graph.processes.get(key) or
                        (graph.frame == key and NAV_ACTION in graph.nodes))
        if key == 'vision':
            return bool(self.vision or graph.processes.get(key))
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
        return bool(self.present('odom') or self.present('map') or self.graph.processes.get('nav')
                    or {NAV_ACTION, '/slicer_nav_arc_guard', '/slicer_velocity_mux'} & self.graph.nodes
                    or self.graph.publishers.get('/cmd_vel'))

    def ready(self, key):
        if key == 'vision':
            return bool(self.vision)
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
            modes = (self.graph.air_mode or '').split('+')
            return any(self.recent(mode) is not None for mode in modes)

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
            elif self.ready(key):
                status = '外部运行中' if external else {'calibrate': '已配准', 'air': '通信运行中'}.get(key, '已就绪')
                result[key] = ('green', status, key != 'calibrate' and not self.stopping_all)
            elif owned or external:
                result[key] = ('yellow', '等待外部状态' if external else '等待就绪', not self.stopping_all)
            else:
                result[key] = ('red', '', not self.stopping_all)
        return result

    def _start(self, key, argv):
        try:
            self.owned[key] = self.spawn(key, argv, self.env)
        except (OSError, ValueError) as error:
            self.report(f'{key} 启动失败 · {error}')
            return False
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
        if (self.graph.online and self.mux.get('manual_hold') is False
                and not any(item in self.owned for item in AUX)
                and self._move_control('stop_nav_hold', 'hold')):
            self.nav_to_stop = key
        else:
            self._signal_stop(key)

    def stop_all(self):
        if self.stopping_all:
            return
        self.switching = False
        self.stopping_all = True
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

    def _calibration_valid(self):
        return (self.ready('sensors') and self.ready('driver') and self.ready('odom') and self._nav_idle()
                and self._safe_to_move() and self.recent('odom_still') is True
                and bool(self.geo) and not self.present('calibrate'))

    def _move_control(self, key, action):
        if '/ugv/control' in self.graph.nodes and not any(item in self.owned for item in AUX):
            self.report('控制台已在操作停车状态，请稍后再试。')
            return False
        return self._start(key, (*CONTROL, action, '--timeout', '4'))

    def control(self, action):
        if self.stopping_all or self.calibrating or self.switching or any(key in self.owned for key in AUX):
            self.report('等待当前流程结束后再操作停车状态。')
        elif action == 'release' and not (self._safe_to_move() and self._nav_idle() and self.ready('driver')):
            self.report('底盘、导航或停车保护尚未就绪。')
        elif self.mux:
            self._move_control(action, action)
        else:
            self.report('等待 Mux 状态。')

    def click(self, key, mode='v1', monitor=True, confirmed=False):
        if self.stopping_all or key in self.stopping:
            return
        if self.local_scan is not None:
            self.graph.processes = self.local_scan()
        if key in self.owned and key != 'calibrate':
            self.stop(key)
            return
        if self.present(key) and key != 'calibrate':
            self.report('该节点由外部终端启动，请在原终端停止。')
            return
        if not self.graph.online and key != 'vision':
            self.report('等待全局 ROS Master。')
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
        if key == 'driver' and (self.present('vision') or 'vision' in self.owned):
            self.report('请先退出视觉抓取窗口，再启动底盘驱动。')
            return
        home = Path(self.env.get('HOME', str(Path.home())))
        gnss = self.env.get('SLICER_GNSS_CONFIG') or str(home / '.config/slicer/gnss.yaml')
        commands = {
            'sensors': ('roslaunch', 'ugv_bringup', 'slicer_field_sensors.launch', f'config_file:={gnss}'),
            'driver': ('roslaunch', 'ugv_bringup', 'slicer_body_driver.launch'),
            'odom': ('roslaunch', 'ugv_bringup', 'slicer_field_navigation.launch', 'geographic:=false'),
            'vision': ('rosrun', 'slicer_visual_grasp', 'run.sh', '--run'),
            'air': ('roslaunch', 'ugv_bringup', 'slicer_air_ground.launch'),
        }
        if key == 'air' and mode == 'v2':
            ip = self.env.get('SLICER_UAV_IP')
            commands[key] = ('roslaunch', 'ag_ugv_task', 'ugv.launch',
                             f'monitor_only:={str(monitor).lower()}', f'start_fkie:={str(bool(ip)).lower()}')
            if ip:
                commands[key] += (f'uav_ip:={ip}',)
        if self._start(key, commands[key]) and key == 'vision' and (self.ready('driver') or self.present('driver')):
            self.report('视觉窗口可预览。抓取动作需要底盘串口，请先停车并关闭底盘驱动。')

    def _calibrate(self, confirmed):
        if self.calibrating or self.ready('calibrate') or self.present('calibrate'):
            self.report('当前配准已完成或正在执行。')
            return
        if not self.board.view(self.clock())['usable']:
            self.report('请等待新鲜 RTK FIXED（gga_quality=4）。')
            return
        if not self._calibration_valid() or any(key in self.owned for key in AUX):
            self.report('请先启动定位、底盘和 odom 导航，确认无任务、急停或手动控制。')
            return
        if not confirmed:
            return 'confirm'
        self.calibrating = True
        if self.mux.get('manual_hold') is True:
            self.calibrating = self._move_control('release', 'release')
        else:
            self._start_registration()

    def _start_registration(self):
        if (self._calibration_valid() and self.mux.get('manual_hold') is False
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
        if not (self.ready('calibrate') and self.ready('sensors') and self.ready('driver')):
            self.report('请先完成配准，并保持定位与底盘运行。')
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
                self._start('map', ('roslaunch', 'ugv_bringup', 'slicer_field_navigation.launch'))
            else:
                self.report('定位、底盘或配准状态变化，暂停切换。')

    def _exit(self, key, status):
        stopping = key in self.stopping
        self.stopping.discard(key)
        if status == 0:
            self.report(f'{key} 已退出')
        elif not stopping:
            self.report(f'{key} 退出（状态 {status}），请查看 ROS 日志。')
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

    def update(self, graph=None, vision=None):
        if graph is not None:
            self.graph = graph
        if vision is not None:
            self.vision = vision
        for key, proc in list(self.owned.items()):
            if key in AUX and self.clock() - self.started[key] > 12. and key not in self.stopping:
                self.report(f'{key} 操作超时，请检查 ROS Master 和 Mux。')
                self._signal_stop(key)
            lines = proc.lines()
            concise = [line for line in lines if any(
                token in line for token in ('ERROR', 'FATAL', 'Traceback', '相机', '[READY]', 'control:', '"reason":'))]
            if concise:
                self.report(concise[-1][:210])
            status = proc.poll()
            if status is not None:
                del self.owned[key]
                del self.started[key]
                self._exit(key, status)
        if self.stopping_all:
            self._advance_stop()
        elif self.switching:
            self._advance_map()
