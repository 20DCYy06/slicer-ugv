#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MODE="${1:-help}";shift || true
export PYTHONDONTWRITEBYTECODE=1
case "$MODE" in
 prepare) exec python3 -s "$ROOT/scripts/prepare_workspace.py";;
 build)
  python3 -s "$ROOT/scripts/prepare_workspace.py"
  mkdir -p "$ROOT/logs"
  "$ROOT/tools/noetic.sh" bash -c 'set -e; catkin_make -j1 -l2; catkin_make install -j1 -l2' 2>&1 | tee "$ROOT/logs/build.log";;
 check) "$ROOT/tools/noetic.sh" bash /workspace/scripts/check.sh 2>&1 | tee "$ROOT/logs/check.log";;
 sensors) exec "$ROOT/tools/slicer_shell.sh" roslaunch ugv_bringup slicer_field_sensors.launch "$@";;
 driver)
  WORKSPACE2="$(cd "$ROOT/../.." && pwd)"
  if [[ -z "${UGV_SERIAL_DEVICE:-}" ]];then
   UGV_SERIAL_DEVICE="$(python3 -s - "$WORKSPACE2" <<'PORT'
import sys
sys.path.insert(0,sys.argv[1]+'/Hardware/Host')
from chassis import device,load_config
print(device(load_config())[0])
PORT
)";export UGV_SERIAL_DEVICE
  fi
  exec "$ROOT/tools/slicer_shell.sh" roslaunch ugv_bringup slicer_body_driver.launch "$@";;
 nav) exec "$ROOT/tools/slicer_shell.sh" roslaunch ugv_bringup slicer_field_navigation.launch "$@";;
 nav-odom) exec "$ROOT/tools/slicer_shell.sh" roslaunch ugv_bringup slicer_field_navigation.launch geographic:=false "$@";;
 tasks) exec "$ROOT/tools/slicer_shell.sh" roslaunch ugv_bringup slicer_air_ground.launch "$@";;
 shell) exec "$ROOT/tools/slicer_shell.sh" "$@";;
 rviz) export UGV_GUI=1;exec "$ROOT/tools/slicer_shell.sh" rviz -d /workspace/install/share/ugv_bringup/rviz/outdoor_navigation.rviz;;
 *) echo 'Usage: ./run.sh {prepare|build|check|sensors|driver|nav-odom|nav|tasks|shell|rviz}';;
esac
