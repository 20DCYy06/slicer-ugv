#!/usr/bin/env bash
# Jetson console display only; no SSH/X11 forwarding and no chassis startup.
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
feature="${1:-point}"
case "$feature" in point|edge) ;; *) echo 'Usage: view_jetson.sh [point|edge] [vision_align.py options]' >&2; exit 2;; esac
if [[ $# -gt 0 ]]; then shift; fi
auth="/run/user/$(id -u)/gdm/Xauthority"
if [[ ! -r "$auth" ]]; then auth="$HOME/.Xauthority"; fi
[[ -r "$auth" && -S /tmp/.X11-unix/X0 ]] || {
  echo 'Jetson display :0 or its Xauthority is unavailable' >&2; exit 2;
}
ros_ip="${ROS_IP:-$(ip -4 route get 1.1.1.1 2>/dev/null | awk '{for(i=1;i<NF;i++)if($i=="src"){print $(i+1);exit}}' || true)}"
ros_ip="${ros_ip:-127.0.0.1}"
capture="$root/field/deployment/20261110-vision"
mkdir -p "$capture"
# Fix :0 explicitly: a DISPLAY inherited from an SSH terminal must not forward.
exec docker run --rm --name slicer-vision-view --network host \
 --cap-drop ALL --security-opt no-new-privileges \
 --user "$(id -u):$(id -g)" --tmpfs /tmp:rw,mode=1777 \
 -e DISPLAY=:0 -e XAUTHORITY=/tmp/vision.xauth -e QT_X11_NO_MITSHM=1 \
 -e ROS_MASTER_URI="${ROS_MASTER_URI:-http://127.0.0.1:11311}" -e "ROS_IP=$ros_ip" \
 -e ROS_HOME=/tmp/slicer-vision-view -e PYTHONDONTWRITEBYTECODE=1 \
 --mount type=bind,src=/tmp/.X11-unix,dst=/tmp/.X11-unix,readonly \
 --mount "type=bind,src=$auth,dst=/tmp/vision.xauth,readonly" \
 --mount "type=bind,src=$root/Test/Vision_adjust,dst=/vision,readonly" \
 --mount "type=bind,src=$capture,dst=/capture" \
 --entrypoint /bin/bash ugv-online-rtk:arm64-20261030 \
 -c 'unset ROS_HOSTNAME; source /opt/ros/noetic/setup.bash; exec python3 -s /vision/vision_align.py --topic /camera/color/image_raw --feature "$1" "${@:2}"' bash "$feature" "$@"
