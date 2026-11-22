#!/usr/bin/env bash
set -eo pipefail

repo=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)
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
if [[ "$repo" != "$workspace/src/slicer-ugv" ]]; then
    echo "Check out this repository at $workspace/src/slicer-ugv before building." >&2
    exit 2
fi

hardware=$(realpath -m -- "${SLICER_HARDWARE_ROOT:-$workspace/deps/slicer_ugv_hardware}")
sdk_source="$workspace/deps/Livox-SDK2"
sdk_install="$workspace/sdk/install"
for required in "$hardware/config.yaml" "$workspace/src/third_party/fast_lio/package.xml" \
    "$workspace/src/third_party/livox_ros_driver2/package.xml" "$sdk_source/CMakeLists.txt"; do
    if [[ ! -r "$required" ]]; then
        printf 'Missing %s. From the workspace root run vcs import . < src/slicer-ugv/dependencies.repos.\n' "$required" >&2
        exit 2
    fi
done

no_rtk_runtime="$workspace/runtime/no_rtk"
rtk_config="$workspace/runtime/rtk_config"
mkdir -p "$workspace/runtime" "$workspace/logs"
export SLICER_HARDWARE_ROOT="$hardware" PYTHONDONTWRITEBYTECODE=1

python3 -s "$repo/src/slicer_navigation_without_mapping/scripts/prepare_runtime.py" \
    --workspace2 "$repo" --hardware-root "$hardware" --output "$no_rtk_runtime"
python3 -s "$repo/tools/rtk/prepare_workspace.py" --hardware-root "$hardware" --config-dir "$rtk_config"

printf 'Catkin sources: %s\nHardware source: %s\nRuntime profiles: %s, %s\n' \
    "$workspace/src" "$hardware" "$no_rtk_runtime" "$rtk_config"
if [[ "$prepare_only" == true ]]; then
    exit 0
fi

cmake -S "$sdk_source" -B "$workspace/sdk/build" -DCMAKE_BUILD_TYPE=Release \
    "-DCMAKE_INSTALL_PREFIX:PATH=$sdk_install"
cmake --build "$workspace/sdk/build" --parallel "${CMAKE_BUILD_PARALLEL_LEVEL:-2}"
cmake --install "$workspace/sdk/build"

set --
source /opt/ros/noetic/setup.bash
exec catkin_make -C "$workspace" --source "$workspace/src" install -DROS_EDITION=ROS1 \
    -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3 \
    "-DSLICER_HARDWARE_ROOT:PATH=$hardware" "-DLIVOX_SDK2_ROOT:PATH=$sdk_install" \
    "-DUGV_BRINGUP_GENERATED_CONFIG_DIR:PATH=$rtk_config" "${catkin_args[@]}"
