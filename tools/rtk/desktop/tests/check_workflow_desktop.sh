#!/usr/bin/env bash
set -eo pipefail
# Real RViz and Qt with simulated data only. Never connect to the field master.
repo=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../../.." && pwd)
cd "$repo"
workspace=${SLICER_NATIVE_WS:-$HOME/slicer_noetic_current}
setup="$workspace/install/setup.bash"
if [[ ! -r "$setup" ]]; then setup="$workspace/devel/setup.bash"; fi
if [[ ! -r "$setup" ]]; then
    printf 'No Noetic overlay at %s\n' "$workspace" >&2
    exit 2
fi
root=$(mktemp -d /tmp/slicer-rtk-workflow-verify.XXXXXXXX)
mkdir -m 700 "$root/session" "$root/ros" "$root/qt"
xvfb=${XVFB:-Xvfb}
"$xvfb" -displayfd 3 -screen 0 1920x1080x24 -nolisten tcp 3>"$root/display" >"$root/xvfb.log" 2>&1 &
server=$!
cleanup() {
    for pid in "${desktop:-}" "${fake:-}" "${core:-}" "$server"; do
        if [[ -n "$pid" ]]; then kill -TERM "$pid" 2>/dev/null || true; fi
    done
    wait 2>/dev/null || true
    printf '验证日志 %s\n' "$root"
}
trap cleanup EXIT
for _ in {1..30}; do
    if [[ -s "$root/display" ]]; then break; fi
    sleep .2
done
if [[ ! -s "$root/display" ]]; then cat "$root/xvfb.log"; exit 1; fi
export DISPLAY=":$(cat "$root/display")" XAUTHORITY="$root/session/auth"
export XDG_RUNTIME_DIR="$root/qt" XDG_CONFIG_HOME="$root/session" ROS_HOME="$root/ros"
source "$setup"
export ROS_IP=127.0.0.1
unset ROS_HOSTNAME
port=$(python3 -c 'import socket; s=socket.socket(); s.bind(("127.0.0.1",0)); print(s.getsockname()[1])')
export ROS_MASTER_URI="http://127.0.0.1:$port" LIBGL_ALWAYS_SOFTWARE=1
export PYTHONPATH="$PWD/src/ugv_navigation/scripts${PYTHONPATH:+:$PYTHONPATH}"
./tools/rtk_desktop_ui.sh >"$root/desktop.log" 2>&1 &
desktop=$!
for _ in {1..40}; do
    if xwininfo -root -tree | rg 'RTK 导航 RViz.*1920x1080.*\+0\+0'; then break; fi
    if ! kill -0 "$desktop" 2>/dev/null; then cat "$root/desktop.log"; exit 1; fi
    sleep .25
done
xwininfo -root -tree | rg 'RTK 导航 RViz.*1920x1080.*\+0\+0'
if xwininfo -root -tree | rg 'RTK 实时状态'; then
    echo 'RTK controls opened a separate window'
    exit 1
fi
rosnode list | rg '^/slicer_rtk_desktop_.*/slicer_rtk_desktop_'
python3 -c 'import tkinter as tk; root=tk.Tk(); root.title("Visual Grasp Fake Tk"); root.geometry("600x420+1320+230"); root.mainloop()' >"$root/fake.log" 2>&1 &
fake=$!
for _ in {1..20}; do
    if xwininfo -root -tree | rg 'Visual Grasp Fake Tk.*600x420\+1320\+230'; then break; fi
    sleep .25
done
xwininfo -root -tree | rg 'Visual Grasp Fake Tk.*600x420\+1320\+230'
kill -INT "$desktop"
wait "$desktop"
desktop=
test -e "/proc/$fake/cmdline"
kill -TERM "$fake"
wait "$fake" || true
fake=
if timeout 2 rosnode list >/dev/null 2>&1; then
    echo 'Master started by UI was not cleaned up'
    exit 1
fi
roscore -p "$port" >"$root/core.log" 2>&1 &
core=$!
for _ in {1..20}; do
    if rosparam set /rtk_desktop_isolated_test true 2>/dev/null; then break; fi
    sleep .25
done
timeout -k 4 55 python3 -B tools/rtk/desktop/tests/check_desktop_ros.py >"$root/rviz-test.log" 2>&1 || {
    tail -n 70 "$root/rviz-test.log"; exit 1;
}
tail -n 12 "$root/rviz-test.log"
printf '%s\n' "ISOLATED_UI_OK $root"
