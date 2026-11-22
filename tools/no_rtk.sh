#!/usr/bin/env bash
set -eo pipefail

repo=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)
mode=${1:-help}
if [[ $# -gt 0 ]]; then shift; fi
workspace=$(realpath -m -- "${SLICER_NATIVE_WS:-${repo%/src/slicer-ugv}}")
runtime="${SLICER_NO_RTK_RUNTIME:-$workspace/runtime/no_rtk}"
state="$runtime"
if [[ -n ${SLICER_STATE_DIR:-} ]]; then state="$SLICER_STATE_DIR/no_rtk"; fi
logs="${SLICER_LOGS_DIR:-$workspace/logs}/no_rtk"

if [[ "$mode" != help && "$mode" != -h && "$mode" != --help &&
      "$repo" != "$workspace/src/slicer-ugv" ]]; then
    printf 'The entrypoint checkout must be %s/src/slicer-ugv.\n' "$workspace" >&2
    exit 2
fi

case "$mode" in
    -h|--help|help)
        echo "Usage: $0 {build|check|nav [--background] [launch arguments]|stop|rviz|goal|patrol|record|diagnose|reset-map|goal-ui}"
        exit 0
        ;;
    build) exec bash "$repo/tools/build_native.sh" "$workspace" "$@" ;;
    check) exec bash "$repo/tools/check_native.sh" "$workspace" "$@" ;;
    stop) ;;
    nav|rviz|goal|patrol|record|diagnose|reset-map|goal-ui)
        setup="$workspace/install/setup.bash"
        if [[ ! -r "$setup" ]]; then
            echo "No installed Catkin overlay at $workspace. Build it with $0 build." >&2
            exit 1
        fi
        arguments=("$@")
        set --
        source /opt/ros/noetic/setup.bash
        source "$setup"
        set -- "${arguments[@]}"
        navigation=$(rospack find slicer_navigation_without_mapping)
        if [[ "$navigation" != "$workspace/install/share/slicer_navigation_without_mapping" ]]; then
            echo "Wrong NoRTK overlay. Expected the installation in $workspace." >&2
            exit 1
        fi
        export SLICER_NO_RTK_RUNTIME="$runtime"
        export SLICER_RUNTIME_DIR="${SLICER_RUNTIME_DIR:-$workspace/runtime}"
        export ROS_HOME="${ROS_HOME:-$workspace/logs/ros}"
        waypoints="$state/waypoints.yaml"
        case "$mode" in
            nav|rviz|patrol|record)
                mkdir -p "$state"
                cp -n -- "$navigation/config/waypoints.yaml" "$waypoints"
                ;;
        esac
        export PYTHONDONTWRITEBYTECODE=1 PYTHONNOUSERSITE=1
        export OMP_NUM_THREADS="${OMP_NUM_THREADS:-1}" OPENBLAS_NUM_THREADS="${OPENBLAS_NUM_THREADS:-1}"
        ;;
    *) echo "Unknown NoRTK command: $mode" >&2; exit 2 ;;
esac

use_local_desktop() {
    local desktop_runtime="/run/user/$UID"
    local desktop_bus="unix:path=$desktop_runtime/bus"
    local name value display='' authority=''
    while IFS='=' read -r name value; do
        case "$name" in
            DISPLAY) display=$value ;;
            XAUTHORITY) authority=$value ;;
        esac
    done < <(XDG_RUNTIME_DIR="$desktop_runtime" DBUS_SESSION_BUS_ADDRESS="$desktop_bus" \
        systemctl --user show-environment 2>/dev/null)
    [[ "$display" =~ ^:[0-9]+(\.[0-9]+)?$ && -r "$authority" ]] || return 1
    DISPLAY="$display" XAUTHORITY="$authority" xset q >/dev/null 2>&1 || return 1
    export DISPLAY="$display" XAUTHORITY="$authority" QT_QPA_PLATFORM=xcb
    export XDG_RUNTIME_DIR="$desktop_runtime" DBUS_SESSION_BUS_ADDRESS="$desktop_bus"
}

case "$mode" in
    goal) exec rosrun slicer_navigation_without_mapping send_goal.py "$@" ;;
    patrol) exec roslaunch slicer_navigation_without_mapping patrol.launch "file:=$waypoints" "$@" ;;
    record) exec rosrun slicer_navigation waypoint_recorder.py "_file:=$waypoints" "$@" ;;
    diagnose) exec rosrun slicer_navigation_without_mapping diagnose_navigation.py "$@" ;;
    reset-map) exec rosrun slicer_navigation_without_mapping reset_map.py "$@" ;;
    rviz|goal-ui)
        if ! use_local_desktop; then
            echo 'No Jetson X11 desktop available. Log in to the local desktop as this user.' >&2
            exit 1
        fi
        if [[ "$mode" == goal-ui ]]; then
            exec rosrun slicer_navigation_without_mapping goal_display.py "$@"
        fi
        exec roslaunch --wait "$navigation/launch/local_desktop.launch" "waypoint_file:=$waypoints" "$@"
        ;;
    nav)
        nav_arguments=("$@")
        mode=foreground
        if [[ ${1:-} == --background ]]; then mode=background; shift; fi
        launch=("$navigation/launch/slicer_nav_without_mapping.launch"
                "runtime_root:=$SLICER_NO_RTK_RUNTIME" "waypoint_file:=$waypoints")
        gui=auto
        for argument in "$@"; do
            if [[ "$argument" == gui:=* ]]; then gui=${argument#gui:=}; fi
        done
        if [[ "${gui,,}" != false ]] && ! use_local_desktop; then
            if [[ "${gui,,}" != auto ]]; then
                echo 'No Jetson X11 desktop available. Log in locally or pass gui:=false.' >&2
                exit 1
            fi
            launch+=(gui:=false)
        fi
        launch+=("$@")
        launch_group=$(python3 - "$workspace" "${launch[@]}" <<'PY'
import grp
import json
import os
import pwd
import shutil
import socket
import sys
from pathlib import Path

import yaml

from roslaunch.config import load_config_default
from roslaunch.core import RLException
from roslaunch.rlutil import resolve_launch_arguments
from roslib.packages import find_node
from rospkg import ResourceNotFound, RosPack

workspace, filename, *arguments = sys.argv[1:]
try:
    path, = resolve_launch_arguments([filename])
    config = load_config_default([(path, arguments)], None, assign_machines=False)
except RLException as exc:
    sys.exit(f'NoRTK launch configuration error in {workspace}: {exc}')

rospack = RosPack()
missing = []
for node in config.nodes:
    try:
        executable = find_node(node.package, node.type, rospack=rospack)
    except ResourceNotFound:
        executable = None
    if not executable:
        missing.append(f'{node.package}/{node.type}')
if missing:
    names = ', '.join(sorted(set(missing)))
    sys.exit(f'Incomplete NoRTK overlay {workspace}: {names}. '
             'Select a complete installation with SLICER_NATIVE_WS or run tools/no_rtk.sh build.')

if '/user_config_path' in config.params:
    filename = config.params['/user_config_path'].value
    try:
        with open(filename) as stream:
            host = json.load(stream)['MID360']['host_net_info']
        for address in sorted({value for key, value in host.items() if key.endswith('_ip') and value}):
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
                try:
                    probe.bind((address, 0))
                except OSError as exc:
                    sys.exit(f'MID360 cannot bind host IP {address}: {exc}. '
                             'Check the Jetson Ethernet IPv4 address and the MID360 network profile.')
    except (OSError, ValueError, KeyError) as exc:
        sys.exit(f'Cannot read MID360 network configuration {filename}: {exc}')

mock = config.params.get('/slicer_c30d_driver/mock')
if mock is not None and not mock.value:
    filename = config.params['/slicer_c30d_driver/host_config_file'].value
    try:
        port = yaml.safe_load(Path(filename).read_text())['serial']['port']
    except (OSError, TypeError, KeyError, yaml.YAMLError) as exc:
        sys.exit(f'Cannot read C30D serial configuration {filename}: {exc}')
    if not isinstance(port, str) or not port:
        sys.exit(f'C30D serial port must be a device path or auto in {filename}.')
    if port != 'auto' and not os.access(port, os.R_OK | os.W_OK):
        try:
            port_group = os.stat(port).st_gid
        except OSError as exc:
            sys.exit(f'C30D serial port unavailable: {port}: {exc}')
        user = pwd.getpwuid(os.getuid())
        active_groups = set(os.getgroups()) | {os.getgid()}
        try:
            dialout = grp.getgrnam('dialout').gr_gid
        except KeyError:
            dialout = None
        if (port_group == dialout and dialout not in active_groups
                and dialout in os.getgrouplist(user.pw_name, user.pw_gid)
                and shutil.which('sg')):
            print('dialout')
        else:
            sys.exit(f'C30D serial port {port} requires read/write access. '
                     'Check the device permissions and this user’s dialout membership.')
PY
        )
        if [[ "$launch_group" == dialout ]]; then
            echo 'Using existing dialout membership for the C30D serial port.' >&2
            # Re-enter before registering a PID so stop can signal roslaunch directly.
            exec python3 -c 'import os, shlex, sys; os.execlp("sg", "sg", "dialout", "-c",
                "exec " + shlex.join(sys.argv[1:]))' "$repo/tools/no_rtk.sh" nav "${nav_arguments[@]}"
        fi
        set -- roslaunch "${launch[@]}"
        ;;
esac

exec python3 - "$mode" "$state" "$logs" "$@" <<'PY'
import fcntl
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

mode, directory, logs, *command = sys.argv[1:]
directory = Path(directory)
pid_file = directory / 'roslaunch.pid'


def start_time(pid):
    if pid <= 0:
        return None
    try:
        fields = Path(f'/proc/{pid}/stat').read_text().rsplit(')', 1)[1].split()
    except (FileNotFoundError, ProcessLookupError):
        return None
    return None if fields[0] == 'Z' else fields[19]


if mode == 'stop' and not directory.is_dir():
    print('No NoRTK navigation session is running for this workspace.')
    sys.exit(0)

directory.mkdir(parents=True, exist_ok=True)
with (directory / 'roslaunch.lock').open('a') as lock:
    fcntl.flock(lock, fcntl.LOCK_EX)
    try:
        owner, started = pid_file.read_text().split()
        pid = int(owner)
    except (FileNotFoundError, ValueError):
        pid, started = 0, ''
    running = start_time(pid) == started

    if mode == 'stop':
        if running:
            try:
                os.kill(pid, signal.SIGINT)
            except ProcessLookupError:
                pass
            deadline = time.monotonic() + 20
            while start_time(pid) == started:
                if time.monotonic() >= deadline:
                    sys.exit(f'NoRTK roslaunch PID {pid} is still stopping.')
                time.sleep(0.1)
            print(f'Stopped NoRTK navigation (PID {pid}).')
        else:
            print('No NoRTK navigation session is running for this workspace.')
        pid_file.unlink(missing_ok=True)
    elif running:
        sys.exit(f'NoRTK navigation is already running (PID {pid}).')
    elif mode == 'foreground':
        pid = os.getpid()
        pid_file.write_text(f'{pid} {start_time(pid)}\n')
        os.execvp(command[0], command)
    else:
        log = Path(logs) / 'roslaunch.log'
        log.parent.mkdir(parents=True, exist_ok=True)
        with log.open('ab') as stream:
            process = subprocess.Popen(command, stdin=subprocess.DEVNULL, stdout=stream,
                                       stderr=subprocess.STDOUT, start_new_session=True)
        started = start_time(process.pid)
        if started is None:
            sys.exit(f'NoRTK roslaunch exited. See {log}.')
        pid_file.write_text(f'{process.pid} {started}\n')
        print(f'Started NoRTK navigation (PID {process.pid}). Log {log}.')
PY
