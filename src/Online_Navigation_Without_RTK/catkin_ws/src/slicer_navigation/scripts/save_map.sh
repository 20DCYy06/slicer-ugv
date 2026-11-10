#!/usr/bin/env bash
# 在现有 Noetic Docker 中执行；宿主入口见 run_navigation.sh save-map NAME。
set -euo pipefail
[[ "${ROS_DISTRO:-}" == noetic ]] || { echo 'Run through run_navigation.sh save-map NAME (Noetic Docker).'; exit 1; }
PKG="$(rospack find slicer_navigation)"
NAME="${1:-map_$(TZ=Asia/Shanghai date +%Y%m%d_%H%M%S)}"
NAME="${NAME%.yaml}"
[[ "$NAME" =~ ^[A-Za-z0-9_-]+$ ]] || { echo 'Map name: letters, digits, underscore or hyphen only'; exit 1; }
MAP_DIR="${SLICER_MAP_DIR:-$PKG/maps}"
mkdir -p "$MAP_DIR"
DEST="$MAP_DIR/$NAME"
echo "Saving /map -> $DEST.yaml and $DEST.pgm"
python3 "$PKG/scripts/map_store.py" "$MAP_DIR" "$NAME"
[[ -s "$DEST.yaml" && -s "$DEST.pgm" ]] || { echo 'Map save failed: missing output'; exit 1; }
if [[ "$MAP_DIR" == "$PKG/maps" ]]; then
  echo "HOST_PATH: /home/potato/Slicer_Workspace2/src/Online_Navigation_Without_RTK/catkin_ws/src/slicer_navigation/maps/$NAME.yaml"
fi
