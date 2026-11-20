#!/usr/bin/env bash
# WSL -> Jetson RGB viewer. Observation by default; explicit app options are forwarded.
set -euo pipefail
feature="${1:-point}"
case "$feature" in point|edge) ;; *) echo 'Usage: view_car.sh [point|edge] [vision_align.py options]' >&2; exit 2;; esac
if [[ $# -gt 0 ]]; then shift; fi
remote_command="$(cat <<'REMOTE'
set -e
[ -n "${DISPLAY:-}" ] || { echo 'SSH X11 forwarding is unavailable' >&2; exit 2; }
exec docker run --rm --name slicer-vision-view --network host \
 --cap-drop ALL --security-opt no-new-privileges \
 --user "$(id -u):$(id -g)" --tmpfs /tmp:rw,mode=1777 \
 -e DISPLAY -e XAUTHORITY=/tmp/vision.xauth -e QT_X11_NO_MITSHM=1 \
 -e QT_XCB_NO_MITSHM=1 -e LIBGL_ALWAYS_SOFTWARE=1 \
 -e ROS_MASTER_URI=http://127.0.0.1:11311 -e ROS_IP=192.168.43.187 \
 -e ROS_HOME=/tmp/slicer-vision-view -e PYTHONDONTWRITEBYTECODE=1 \
 --mount "type=bind,src=$HOME/.Xauthority,dst=/tmp/vision.xauth,readonly" \
 --mount type=bind,src=/home/potato/slicer_workspace2_codex/Test/Vision_adjust,dst=/vision,readonly \
 --mount type=bind,src=/home/potato/slicer_workspace2_codex/field/deployment/20261110-vision,dst=/capture \
 --entrypoint /bin/bash ugv-online-rtk:arm64-20261030 \
 -c 'source /opt/ros/noetic/setup.bash; exec python3 -s /vision/vision_align.py --topic /camera/color/image_raw --feature "$1" "${@:2}"' bash
REMOTE
)"
printf -v quoted_arguments ' %q' "$feature" "$@"
exec ssh -Y -C cup-jetson "$remote_command$quoted_arguments"
