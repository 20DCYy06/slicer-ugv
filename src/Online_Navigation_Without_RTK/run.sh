#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE2="$(cd "$ROOT/../.." && pwd)"
PKG="$ROOT/catkin_ws/src/slicer_navigation_without_mapping"
MODE="${1:-help}"; shift || true
read -r MASTER IP IMAGE NAME < <(python3 -s - "$PKG/config/runtime.yaml" <<'CONFIG'
import sys,yaml
c=yaml.safe_load(open(sys.argv[1]));print(c['ros_master_uri'],c['ros_ip'],c['docker_image'],c['container_name'])
CONFIG
)
export PYTHONDONTWRITEBYTECODE=1
ENV=(-e WORKSPACE2_ROOT=/workspace2 -e PYTHONDONTWRITEBYTECODE=1 -e PYTHONNOUSERSITE=1 -e PYTHONUNBUFFERED=1 -e OMP_NUM_THREADS=1 -e OPENBLAS_NUM_THREADS=1)
if [[ "$MODE" == help ]]; then
 echo 'Usage: ./run.sh {build|check|nav [--background] [gui:=false] [start_control:=false]|goal --x X --y Y [--relative]|patrol|record|goal-ui|reset-map|diagnose [--duration 30]|stop}'
 exit
fi
if [[ "$MODE" == stop ]]; then exec docker stop -t 8 "$NAME"; fi
if [[ "$MODE" == build ]]; then
 mkdir -p "$ROOT/logs"
 docker run --rm --network none --user "$(id -u):$(id -g)" "${ENV[@]}" -e HOME=/tmp \
 -v "$ROOT:/slicer" -v "$WORKSPACE2/Hardware:/workspace2/Hardware:ro" -w /slicer "$IMAGE" bash -c \
 'set -e; source /opt/ros/noetic/setup.bash; catkin_make -C catkin_ws -DROS_EDITION=ROS1 -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3 -j1 -l2' 2>&1 | tee "$ROOT/logs/build.log"
 exit
fi
if [[ "$MODE" == check ]]; then
 [[ -f "$ROOT/catkin_ws/devel/setup.bash" ]] || { echo '先运行 ./run.sh build';exit 1; }
 mkdir -p "$ROOT/logs/check"
 docker run --rm --network none --user "$(id -u):$(id -g)" "${ENV[@]}" \
 -e HOME=/tmp/home -e ROS_HOME=/tmp/ros -e ROS_LOG_DIR=/tmp/ros-log -e ROS_MASTER_URI=http://127.0.0.1:11311 -e ROS_IP=127.0.0.1 \
 -v "$ROOT:/slicer:ro" -v "$WORKSPACE2/Hardware:/workspace2/Hardware:ro" -v "$ROOT/logs/check:/tmp" -w /slicer "$IMAGE" bash -c \
 'set -e; mkdir -p "$HOME" "$ROS_HOME" "$ROS_LOG_DIR"; source /opt/ros/noetic/setup.bash; source catkin_ws/devel/setup.bash; exec python3 catkin_ws/src/slicer_navigation_without_mapping/tests/check_system.py' 2>&1 | tee "$ROOT/logs/check.log"
 exit
fi
if [[ "$MODE" == goal || "$MODE" == patrol || "$MODE" == diagnose || "$MODE" == reset-map || "$MODE" == goal-ui || "$MODE" == record ]]; then
 TTY=(-i);[[ ! -t 0 || ! -t 1 ]] || TTY+=(-t)
 case "$MODE" in
 goal) CMD=(rosrun slicer_navigation_without_mapping send_goal.py "$@");;
 patrol) CMD=(roslaunch slicer_navigation_without_mapping patrol.launch "$@");;
 diagnose) CMD=(rosrun slicer_navigation_without_mapping diagnose_navigation.py "$@");;
 reset-map) CMD=(rosrun slicer_navigation_without_mapping reset_map.py "$@");;
 goal-ui) TTY=(-d);CMD=(rosrun slicer_navigation_without_mapping goal_display.py);;
 record) CMD=(rosrun slicer_navigation waypoint_recorder.py _file:=/slicer/catkin_ws/src/slicer_navigation_without_mapping/config/waypoints.yaml "$@");;
 esac
 exec docker exec "${TTY[@]}" "$NAME" bash -c 'source /opt/ros/noetic/setup.bash; source /slicer/catkin_ws/devel/setup.bash; exec "$@"' bash "${CMD[@]}"
fi
[[ "$MODE" == nav ]] || { echo "Unknown mode: $MODE";exit 2; }
BACKGROUND=();if [[ "${1:-}" == --background ]]; then BACKGROUND=(-d);shift;fi
for old in slicer-navigation slicer-mapping slicer-online-navigation "$NAME"; do
 if docker ps --format '{{.Names}}' | grep -Fxq "$old";then echo "已有导航容器 $old，请先手动停止以避免设备/TF冲突。";exit 1;fi
done
[[ -f "$ROOT/catkin_ws/devel/setup.bash" ]] || { echo '先运行 ./run.sh build';exit 1; }
mkdir -p "$PKG/.runtime" "$PKG/logs"
python3 -s "$PKG/scripts/prepare_runtime.py" --root "$ROOT" --output "$PKG/.runtime" --workspace2 "$WORKSPACE2"
ARGS=(--rm --init --network host --name "$NAME" --user "$(id -u):$(id -g)" "${ENV[@]}" \
 -e HOME=/tmp/online-home -e ROS_HOME=/tmp/online-ros -e ROS_LOG_DIR=/tmp/online-log \
 -e ROS_MASTER_URI="${ROS_MASTER_URI:-$MASTER}" -e ROS_IP="${ROS_IP:-$IP}" \
 -v "$ROOT:/slicer:ro" -v "$PKG/config:/slicer/catkin_ws/src/slicer_navigation_without_mapping/config" \
 -v "$WORKSPACE2/Hardware:/workspace2/Hardware:ro" -v "$PKG/logs:/tmp/online-log" -w /slicer)
if [[ " $* " != *" start_control:=false "* && " $* " != *" control_mock:=true "* ]]; then
 PORT="$(python3 -s - "$WORKSPACE2" <<'PORT'
import sys
sys.path.insert(0,sys.argv[1]+'/Hardware/Host')
from chassis import load_config,device
print(device(load_config())[0])
PORT
)"
 [[ -c "$PORT" ]] || { echo '找不到Workspace2配置的C30D';exit 1; }
 ARGS+=(--device "$PORT" --group-add "$(stat -c %g "$PORT")" -v /run/udev:/run/udev:ro)
fi
if [[ " $* " != *" gui:=false "* ]];then
 AUTH="/run/user/$(id -u)/gdm/Xauthority";[[ -r "$AUTH" ]] || AUTH="${XAUTHORITY:-$HOME/.Xauthority}"
 [[ -r "$AUTH" ]] || { echo '请先登录Jetson桌面，或使用 gui:=false';exit 1; }
 DISPLAY=:0 XAUTHORITY="$AUTH" "$PKG/scripts/low_display.sh" || echo '保持当前桌面分辨率'
 ARGS+=(-e DISPLAY=:0 -e XAUTHORITY=/tmp/online.xauth -e QT_X11_NO_MITSHM=1 -v /tmp/.X11-unix:/tmp/.X11-unix -v "$AUTH:/tmp/online.xauth:ro")
fi
exec docker run "${ARGS[@]}" "${BACKGROUND[@]}" "$IMAGE" bash -c \
 'set -e; mkdir -p "$HOME" "$ROS_HOME" "$ROS_LOG_DIR"; source /opt/ros/noetic/setup.bash; source catkin_ws/devel/setup.bash; PORT="${ROS_MASTER_URI##*:}"; PORT="${PORT%/}"; exec roslaunch -p "$PORT" slicer_navigation_without_mapping slicer_nav_without_mapping.launch "$@"' bash "$@"
