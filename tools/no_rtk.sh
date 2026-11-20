#!/usr/bin/env bash
set -eo pipefail

repo=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
workspace=$(realpath -m -- "${SLICER_NATIVE_WS:-$HOME/slicer_noetic_dev}")
runtime="$workspace/runtime/no_rtk"
mode=${1:-help}
if [[ $# -gt 0 ]]; then shift; fi

case "$mode" in
    -h|--help|help)
        echo "Usage: $0 {build|check|nav [--background] [launch arguments]|stop|goal|patrol|record|diagnose|reset-map|goal-ui}"
        exit 0
        ;;
    build) exec bash "$repo/tools/build_native.sh" "$workspace" "$@" ;;
    check) exec bash "$repo/tools/check_native.sh" "$workspace" "$@" ;;
    stop) ;;
    nav|goal|patrol|record|diagnose|reset-map|goal-ui)
        setup="$workspace/install/setup.bash"
        if [[ ! -f "$setup" ]]; then setup="$workspace/devel/setup.bash"; fi
        if [[ ! -f "$setup" ]]; then
            echo "No Catkin overlay at $workspace. Build it with $0 build." >&2
            exit 1
        fi
        arguments=("$@")
        set --
        source "$setup"
        set -- "${arguments[@]}"
        export WORKSPACE2_ROOT="$repo"
        export SLICER_NO_RTK_RUNTIME="${SLICER_NO_RTK_RUNTIME:-$runtime}"
        export PYTHONDONTWRITEBYTECODE=1 PYTHONNOUSERSITE=1
        export OMP_NUM_THREADS="${OMP_NUM_THREADS:-1}" OPENBLAS_NUM_THREADS="${OPENBLAS_NUM_THREADS:-1}"
        ;;
    *) echo "Unknown NoRTK command: $mode" >&2; exit 2 ;;
esac

waypoints="$repo/src/slicer_navigation_without_mapping/config/waypoints.yaml"
case "$mode" in
    goal) exec rosrun slicer_navigation_without_mapping send_goal.py "$@" ;;
    patrol) exec roslaunch slicer_navigation_without_mapping patrol.launch "file:=$waypoints" "$@" ;;
    record) exec rosrun slicer_navigation waypoint_recorder.py "_file:=$waypoints" "$@" ;;
    diagnose) exec rosrun slicer_navigation_without_mapping diagnose_navigation.py "$@" ;;
    reset-map) exec rosrun slicer_navigation_without_mapping reset_map.py "$@" ;;
    goal-ui) exec rosrun slicer_navigation_without_mapping goal_display.py "$@" ;;
    nav)
        mode=foreground
        if [[ ${1:-} == --background ]]; then mode=background; shift; fi
        set -- roslaunch slicer_navigation_without_mapping slicer_nav_without_mapping.launch \
            "workspace2_root:=$repo" "runtime_root:=$SLICER_NO_RTK_RUNTIME" "$@"
        ;;
esac

exec python3 - "$mode" "$runtime" "$@" <<'PY'
import fcntl
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

mode, directory, *command = sys.argv[1:]
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
        log = directory / 'roslaunch.log'
        with log.open('ab') as stream:
            process = subprocess.Popen(command, stdin=subprocess.DEVNULL, stdout=stream,
                                       stderr=subprocess.STDOUT, start_new_session=True)
        started = start_time(process.pid)
        if started is None:
            sys.exit(f'NoRTK roslaunch exited. See {log}.')
        pid_file.write_text(f'{process.pid} {started}\n')
        print(f'Started NoRTK navigation (PID {process.pid}). Log {log}.')
PY
