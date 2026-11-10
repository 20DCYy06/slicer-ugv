#!/usr/bin/env bash
# WSL observation only: Docker/WSLg environment plus one RViz preset.
set -euo pipefail

case "${1:-perception}" in
  perception) ugv_preset=small_obstacle.rviz ;;
  navigation) ugv_preset=odom_navigation.rviz ;;
  online) ugv_preset=online_navigation.rviz ;;
  -h|--help)
    printf 'Usage: %s [perception|navigation|online] [MASTER_IP|http://HOST:PORT]\n' "$0"
    printf '  perception (default): point clouds, scan, costmaps and footprint\n'
    printf '  navigation: goals, global/local plans, odom and costmaps\n'
    printf '  online: map, depth obstacles, goals/plans, odom and costmaps\n'
    printf 'Omit the address in a terminal to choose it interactively; Enter keeps the default.\n'
    printf 'Explicit address overrides ROS_MASTER_URI and rviz_observer.env for this run only.\n'
    printf 'Uses ROS_IP when set; otherwise selects it from the route to the chosen master.\n'
    exit 0 ;;
  *) printf 'Choose perception, navigation or online. See --help.\n' >&2; exit 2 ;;
esac
if [[ $# -gt 2 ]]; then
  printf 'Use one preset and an optional master address. See --help.\n' >&2
  exit 2
fi

ugv_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
# Observer-only defaults: never inherit the Jetson runtime image or its ROS_IP.
ugv_observer_config="$ugv_root/src/ugv_bringup/config/rviz_observer.env"
if [[ -f "$ugv_observer_config" ]]; then source "$ugv_observer_config"; fi
ugv_image="${UGV_NOETIC_IMAGE:-industrial-cup/nav-slicer:20261023-dwa}"
ugv_master="${ROS_MASTER_URI:-${UGV_RVIZ_MASTER_URI:-http://192.168.10.33:11311}}"
ugv_master_choice="${2:-}"
if [[ $# -lt 2 && -t 0 ]]; then
  read -r -p "ROS master IP/URI [$ugv_master]: " ugv_master_choice || ugv_master_choice=""
fi
if [[ -n "$ugv_master_choice" ]]; then
  case "$ugv_master_choice" in
    http://*) ugv_master="$ugv_master_choice" ;;
    *:*) ugv_master="http://$ugv_master_choice" ;;
    *) ugv_master="http://$ugv_master_choice:11311" ;;
  esac
fi
ugv_ip="${ROS_IP:-}"
if [[ -z "$ugv_ip" ]]; then
  ugv_master_host="${ugv_master#*://}"
  ugv_master_host="${ugv_master_host%%:*}"
  ugv_master_host="${ugv_master_host%%/*}"
  ugv_ip="$(ip -4 route get "$ugv_master_host" | awk '{for(i=1;i<NF;i++) if($i=="src") {print $(i+1); exit}}')"
  if [[ -z "$ugv_ip" ]]; then
    printf 'Cannot select observer IP. Set ROS_IP to the WSL address reachable from the car.\n' >&2
    exit 2
  fi
fi

printf 'RViz: %s | master=%s | observer=%s\n' "$ugv_preset" "$ugv_master" "$ugv_ip"
exec docker run --rm --network host \
  --cap-drop ALL --security-opt no-new-privileges \
  --user "$(id -u):$(id -g)" \
  -e DISPLAY -e QT_QPA_PLATFORM=xcb -e QT_XCB_NO_MITSHM=1 \
  -e LIBGL_ALWAYS_SOFTWARE=1 -e ROS_HOME=/tmp/ugv-rviz-ros \
  -e "ROS_MASTER_URI=$ugv_master" -e "ROS_IP=$ugv_ip" -e ROS_HOSTNAME= \
  --mount type=bind,src=/tmp/.X11-unix,dst=/tmp/.X11-unix,readonly \
  --mount "type=bind,src=$ugv_root/src/ugv_bringup/rviz,dst=/rviz,readonly" \
  --entrypoint /bin/bash "$ugv_image" \
  -c 'set -e; unset ROS_HOSTNAME; source /opt/ros/noetic/setup.bash; exec rviz -d "$1" __name:=ugv_rviz_observe' \
  bash "/rviz/$ugv_preset"
