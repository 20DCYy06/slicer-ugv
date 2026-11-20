#!/usr/bin/env bash
set -eo pipefail

repo=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
prepare_only=false
if [[ ${1:-} == --prepare-only ]]; then
    prepare_only=true
    shift
fi
if [[ $# == 0 || ${1:-} == --help || ${1:-} == -h ]]; then
    echo "Usage: bash tools/build_native.sh [--prepare-only] WORKSPACE [catkin_make options]"
    exit 0
fi

workspace=$(realpath -m -- "$1")
shift
catkin_args=("$@")
if [[ "$workspace" == "$repo" || "$workspace" == "$repo/"* ]]; then
    echo "Choose a Catkin workspace outside the source repository." >&2
    exit 2
fi

sources="$workspace/src"
no_rtk="$repo/src/Online_Navigation_Without_RTK"
rtk="$repo/src/Online_Navigation_with_RTK"
ag="$repo/ag_v2_ws/src/AG_Client"
no_rtk_runtime="$workspace/runtime/no_rtk"
rtk_config="$workspace/runtime/rtk_config"
mkdir -p "$sources"

link_source() {
    local from=$1 to=$2
    if [[ ! -L "$to" || $(readlink -- "$to") != "$from" ]]; then
        ln -sT -- "$from" "$to"
    fi
}

for package in fast_lio slicer_bringup slicer_navigation slicer_navigation_without_mapping slicer_wasd_control; do
    link_source "$no_rtk/catkin_ws/src/$package" "$sources/$package"
done
for package in cup_gnss slicer_air_ground_msgs ugv_bringup ugv_navigation ugv_slicer_control; do
    link_source "$rtk/src/$package" "$sources/$package"
done
link_source "$repo/src/conmunication" "$sources/ugv_communication"
for package in ag_ugv_task ag_uav_client; do
    link_source "$ag/$package" "$sources/$package"
done

livox="$no_rtk/catkin_ws/src/livox_ros_driver2"
livox_package="$sources/livox_ros_driver2"
mkdir -p "$livox_package"
for path in CMakeLists.txt 3rdparty cmake config launch_ROS1 msg src; do
    link_source "$livox/$path" "$livox_package/$path"
done
install -m 644 -- "$livox/package_ROS1.xml" "$livox_package/package.xml"

for package in "$sources"/*; do
    if [[ ! -f "$package/package.xml" && "$package" != "$sources/CMakeLists.txt" ]]; then
        echo "Missing package manifest at $package/package.xml" >&2
        exit 1
    fi
done

export PYTHONDONTWRITEBYTECODE=1
python3 -s "$no_rtk/catkin_ws/src/slicer_navigation_without_mapping/scripts/prepare_runtime.py" \
    --root "$no_rtk" --output "$no_rtk_runtime" --workspace2 "$repo"
python3 -s "$rtk/scripts/prepare_workspace.py" --config-dir "$rtk_config"

printf 'Catkin sources: %s\nNoRTK runtime_root: %s\nNoRTK workspace2_root: %s\n' \
    "$sources" "$no_rtk_runtime" "$repo"
if [[ "$prepare_only" == true ]]; then
    exit 0
fi

set --
source /opt/ros/noetic/setup.bash
exec catkin_make -C "$workspace" install -DROS_EDITION=ROS1 -DCMAKE_BUILD_TYPE=Release \
    -DPYTHON_EXECUTABLE=/usr/bin/python3 -DUGV_BRINGUP_GENERATED_CONFIG_DIR="$rtk_config" "${catkin_args[@]}"
