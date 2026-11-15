#!/usr/bin/env bash
# 不启动导航、底盘或目标；只管理Workspace2通信实例，不接管旧ugv-*容器。
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE2="$(cd "$HERE/../.." && pwd)"
source "$HERE/config/network.env"
mode="${1:-status}"
MASTER="${ROS_MASTER_URI:-http://$UGV_ROS_IP:11311}"
POINTS="${UGV_WAYPOINT_DIR:-$WORKSPACE2/src/Online_Navigation_Without_RTK/catkin_ws/src/slicer_navigation_without_mapping/config}"
LEGACY_WAYPOINTS="${UGV_ENABLE_LEGACY_WAYPOINTS:-false}"
case "$LEGACY_WAYPOINTS" in true|false) ;; *) echo 'UGV_ENABLE_LEGACY_WAYPOINTS must be true or false';exit 2;; esac
RTK_INSTALL="$WORKSPACE2/src/Online_Navigation_with_RTK/install"
RTK_MOUNTS=()
# 只读复用我方已构建的消息包；不自动构建，也不挂载队友/旧工程。
if [[ -f "$RTK_INSTALL/setup.bash" ]]; then
 RTK_MOUNTS=(-v "$RTK_INSTALL:/rtk_install:ro")
fi
COMMON=(--init --network host --restart unless-stopped --user "$(id -u):$(id -g)"
 -e "ROS_MASTER_URI=$MASTER" -e "ROS_IP=$UGV_ROS_IP" -e ROS_HOSTNAME= -e PYTHONDONTWRITEBYTECODE=1
 -e PYTHONNOUSERSITE=1 -e PYTHONUNBUFFERED=1 -e HOME=/tmp/ugv-communication
 -e ROS_LOG_DIR=/tmp/ugv-communication-log -e ROS_PACKAGE_PATH=/communication:/opt/ros/noetic/share
 -v "$HERE:/communication:ro" "${RTK_MOUNTS[@]}")
start_one() {
 local name="$COMM_CONTAINER_PREFIX-$1";shift
 if docker inspect "$name" >/dev/null 2>&1;then
  if [[ "$(docker inspect "$name" --format '{{.State.Running}}')" == true ]];then echo "$name already running";else docker start "$name";fi
 else docker run -d --name "$name" "${COMMON[@]}" "$@";fi
}
relay(){ start_one state-relay "$FKIE_IMAGE" bash -c 'unset ROS_HOSTNAME; source /opt/ros/noetic/setup.bash; if [[ -f /rtk_install/setup.bash ]]; then source /rtk_install/setup.bash --extend; fi; export ROS_NAMESPACE=/ugv/communication; exec python3 -B /communication/relay.py'; }
case "$mode" in
 check)
  mkdir -p "$HERE/logs/check"
  docker run --rm --network none --user "$(id -u):$(id -g)" -e HOME=/tmp -e ROS_HOME=/tmp/ros -e ROS_LOG_DIR=/tmp/ros-log \
   -e ROS_MASTER_URI=http://127.0.0.1:11311 -e ROS_IP=127.0.0.1 -e PYTHONDONTWRITEBYTECODE=1 \
   -e ROS_PACKAGE_PATH=/communication:/opt/ros/noetic/share -v "$HERE:/communication:ro" -v "$HERE/logs/check:/tmp" "$FKIE_IMAGE" bash -c \
   'source /opt/ros/noetic/setup.bash; export ROS_PACKAGE_PATH=/communication:$ROS_PACKAGE_PATH; exec python3 -B /communication/verify_relay.py' 2>&1 | tee "$HERE/logs/check.log";;
 start-relay) relay;;
 start)
  # 复用已经运行的本机Master；不创建第二个11311 Master。
  if ! python3 - "$MASTER" <<'MASTER_CHECK'
import sys,xmlrpc.client
try:sys.exit(0 if xmlrpc.client.ServerProxy(sys.argv[1]).getPid('/workspace2_comm_check')[0]==1 else 1)
except OSError:sys.exit(1)
MASTER_CHECK
  then start_one roscore "$FKIE_IMAGE" bash -c 'unset ROS_HOSTNAME; source /opt/ros/noetic/setup.bash; exec roscore';fi
  start_one discovery -e "ROBOT_HOSTS=['$UAV_ROS_IP']" "$FKIE_IMAGE" bash -c \
   'unset ROS_HOSTNAME; source /opt/ros/noetic/setup.bash; if [[ -f /rtk_install/setup.bash ]]; then source /rtk_install/setup.bash --extend; fi; until rosparam get /rosdistro >/dev/null 2>&1;do sleep 2;done; exec rosrun fkie_master_discovery master_discovery "_robot_hosts:=$ROBOT_HOSTS"'
  SYNC_MOUNTS=()
  if [[ "$LEGACY_WAYPOINTS" == true ]]; then
   [[ -d "$POINTS" ]] || { echo "路点目录不存在：$POINTS";exit 1; }
   SYNC_MOUNTS=(-v "$POINTS:/navigation")
  fi
  start_one sync "${SYNC_MOUNTS[@]}" "$FKIE_IMAGE" bash -c \
   'unset ROS_HOSTNAME; source /opt/ros/noetic/setup.bash; if [[ -f /rtk_install/setup.bash ]]; then source /rtk_install/setup.bash --extend; fi; export ROS_PACKAGE_PATH=/communication:$ROS_PACKAGE_PATH; until rosservice list 2>/dev/null | grep -qx /master_discovery/list_masters;do sleep 2;done; exec roslaunch /communication/launch/fkie_ugv_sync.launch "$@"' bash "enable_legacy_waypoints:=$LEGACY_WAYPOINTS"
  relay;;
 reload-sync) docker restart "$COMM_CONTAINER_PREFIX-sync";;
 status) docker ps -a --filter "name=$COMM_CONTAINER_PREFIX-" --format 'table {{.Names}}\t{{.Image}}\t{{.Status}}';;
 stop) for part in state-relay sync discovery;do name="$COMM_CONTAINER_PREFIX-$part"; if docker inspect "$name" >/dev/null 2>&1;then docker stop "$name";fi;done;;
 build) docker build --network host --build-arg "BASE_IMAGE=$FKIE_BASE_IMAGE" -f "$HERE/Dockerfile.fkie" -t "$FKIE_IMAGE" "$HERE";;
 *) echo 'Usage: ./run.sh {check|start|start-relay|reload-sync|status|stop|build}';exit 2;;
esac
