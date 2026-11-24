"""Read the global ROS graph and identify local launches before starting another."""
from dataclasses import dataclass, field
import ipaddress
import os
from pathlib import Path
import socket
from urllib.parse import urlparse
from xmlrpc.client import Error, ServerProxy, Transport


CALLER_ID = '/slicer_rtk_desktop_probe'
NAV_ACTION = '/ugv/move_base'
LAUNCHES = {
    'slicer_sensors.launch': ('sensors',),
    'slicer_geographic.launch': ('rtk',),
    'slicer_gnss.launch': ('rtk',),
    'slicer_body_driver.launch': ('driver',),
    'slicer_driver.launch': ('driver',),
    'slicer_odom.launch': ('nav',),
    'slicer_navigation_runtime.launch': ('navigation', 'nav'),
    'slicer_online.launch': ('nav',),
    'slicer_air_ground.launch': ('air', 'v1'),
}


@dataclass
class Graph:
    online: bool = False
    nodes: set = field(default_factory=set)
    processes: dict = field(default_factory=dict)
    frame: str = None
    publishers: dict = field(default_factory=dict)
    subscribers: dict = field(default_factory=dict)
    air_mode: str = None


def get_master_uri(env=None):
    env = os.environ if env is None else env
    return env.get('ROS_MASTER_URI') or 'http://localhost:11311'


def master_local(env=None):
    """Only auto-start roscore if the configured master address belongs to this Jetson."""
    try:
        uri = urlparse(get_master_uri(env))
        if uri.scheme != 'http' or not uri.hostname:
            return False
        if uri.hostname == 'localhost':
            return True
        addresses = socket.getaddrinfo(uri.hostname, uri.port or 11311, type=socket.SOCK_DGRAM)
    except (OSError, ValueError):
        return False
    for family, kind, proto, _, address in addresses:
        if ipaddress.ip_address(address[0]).is_loopback:
            return True
        try:
            with socket.socket(family, kind, proto) as endpoint:
                # Bind a temporary port on this interface. This sends no network traffic.
                endpoint.bind((address[0], 0) + address[2:])
                return True
        except OSError:
            continue
    return False


def _command(argv):
    """Remove an interpreter prefix without treating shell text as argv."""
    name = Path(argv[0]).name if argv else ''
    if name.startswith('python') or name in ('bash', 'sh'):
        for i, arg in enumerate(argv[1:], 1):
            if arg == '-m':
                return argv[i + 1:]
            if arg == '-c':
                return []
            if not arg.startswith('-'):
                return argv[i:]
        return []
    return argv


def _process_keys(argv):
    invocation = _command(argv)
    if not invocation:
        return ()
    program, *args = invocation
    name = Path(program).name
    if name == 'rosrun' and len(args) >= 2 and not args[0].startswith('-'):
        program = str(Path(args[0]) / args[1])
        args = args[2:]
        name = Path(program).name
    if name == 'roslaunch':
        if any(arg in ('--nodes', '--files', '--dump-params', '--find-node') for arg in args):
            return ()
        path = next((Path(arg) for arg in args if arg.endswith('.launch')), Path())
        launch = path.name
        if launch == 'slicer_field_sensors.launch':
            groups = []
            if 'start_local_sensors:=false' not in args:
                groups.append('sensors')
            if 'geographic:=false' not in args:
                groups.append('rtk')
            return tuple(groups)
        if launch == 'slicer_field_navigation.launch':
            if 'mapping:=true' in args:
                return ('nav',)
            frame = 'odom' if 'geographic:=false' in args else 'map'
            return frame, 'nav'
        if launch == 'ugv.launch' and ('ag_ugv_task' in args or path.parent.parent.name == 'ag_ugv_task'):
            return 'air', 'v2'
        return LAUNCHES.get(launch, ())
    if name == 'geographic.py' and 'initialize' in args:
        return ('calibrate',)
    if (name == 'slicer_visual_grasp.app'
            or name == 'run.sh' and Path(program).parent.name == 'slicer_visual_grasp'):
        return () if set(args) & {'--check', '--test', '--ui-test', '--help', '-h'} else ('vision',)
    if name == 'air_ground_node.py':
        return 'air', 'v1'
    if name == 'ag_ugv_task_node.py':
        return 'air', 'v2'
    if name in ('c30d_driver_node.py', 'run_b15_servo.sh', 'run_combined_gamepad.sh',
                'combined_gamepad.py') or name == 'keyboard.py' and Path(program).parent.name == 'b15_servo':
        return ('driver',)
    if name == 'navigation_runtime.py':
        return ('navigation', 'nav')
    if name == 'move_base':
        return ('nav',)
    return ()


def process_snapshot(root=Path('/proc'), uid=None):
    """Return process group IDs for same-user programs, including vision with no ROS node."""
    uid = os.geteuid() if uid is None else uid
    groups = {}
    for process in root.iterdir():
        if not process.name.isdecimal():
            continue
        try:
            if process.stat().st_uid != uid:
                continue
            argv = [os.fsdecode(arg) for arg in (process / 'cmdline').read_bytes().split(b'\0') if arg]
            keys = _process_keys(argv)
            if keys:
                group = os.getpgid(int(process.name))
                for key in keys:
                    groups.setdefault(key, set()).add(group)
        except OSError:
            # Processes can exit or become unreadable during the snapshot.
            continue
    return groups


class _TimedTransport(Transport):
    def make_connection(self, host):
        connection = super().make_connection(host)
        connection.timeout = .5
        return connection


def _air_mode(graph):
    v1 = ('/air_ground_task' in graph.nodes or graph.processes.get('v1')
          or graph.publishers.get('/air_ground/state'))
    v2 = ('/ag_ugv_task' in graph.nodes or graph.processes.get('v2')
          or graph.publishers.get('/air_ground/v2/state'))
    if v1 and v2:
        return 'v1+v2'
    if v1:
        return 'v1'
    return 'v2' if v2 else None


def discover(env=None):
    """Read registrations and the active planner frame from the standard ROS master."""
    graph = Graph(processes=process_snapshot())
    try:
        with ServerProxy(get_master_uri(env), transport=_TimedTransport()) as master:
            code, _, state = master.getSystemState(CALLER_ID)
            if code == 1:
                publishers, subscribers, services = ({key: set(nodes) for key, nodes in values}
                                                     for values in state)
                graph.publishers = publishers
                graph.subscribers = subscribers
                graph.nodes = {node for registrations in (publishers, subscribers, services)
                               for names in registrations.values() for node in names}
                graph.online = True
                if NAV_ACTION in graph.nodes:
                    code, _, frame = master.getParam(CALLER_ID, NAV_ACTION + '/global_costmap/global_frame')
                    if code == 1 and frame in ('odom', 'map'):
                        graph.frame = frame
    except (Error, OSError, TypeError, ValueError):
        # ROS Master may be absent, restarting or unreachable during a workflow transition.
        pass
    graph.air_mode = _air_mode(graph)
    return graph
