#!/usr/bin/env bash
# 在已加载的 Noetic overlay 中保存当前地图。
set -euo pipefail
[[ "${ROS_DISTRO:-}" == noetic ]] || { echo 'Source the Noetic Catkin overlay to save the map.'; exit 1; }
PKG="$(rospack find slicer_navigation)"
NAME="${1:-map_$(TZ=Asia/Shanghai date +%Y%m%d_%H%M%S)}"
NAME="${NAME%.yaml}"
[[ "$NAME" =~ ^[A-Za-z0-9_-]+$ ]] || { echo 'Map name: letters, digits, underscore or hyphen only'; exit 1; }
MAP_DIR="${SLICER_MAP_DIR:-$PKG/maps}"
mkdir -p "$MAP_DIR"
DEST="$MAP_DIR/$NAME"
echo "Saving /map -> $DEST.yaml and $DEST.pgm"
rosrun slicer_navigation map_store.py "$MAP_DIR" "$NAME"
[[ -s "$DEST.yaml" && -s "$DEST.pgm" ]] || { echo 'Map save failed: missing output'; exit 1; }
