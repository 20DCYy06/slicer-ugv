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

if [[ ! -r "$repo/Hardware/config.yaml" ]]; then
    echo "Hardware/config.yaml is missing. Run: git submodule update --init -- Hardware" >&2
    exit 1
fi

no_rtk_runtime="$workspace/runtime/no_rtk"
rtk_config="$workspace/runtime/rtk_config"
mkdir -p "$workspace"

export PYTHONDONTWRITEBYTECODE=1
python3 -s "$repo/src/slicer_navigation_without_mapping/scripts/prepare_runtime.py" \
    --workspace2 "$repo" --output "$no_rtk_runtime"
python3 -s "$repo/tools/rtk/prepare_workspace.py" --config-dir "$rtk_config"

printf 'Catkin sources: %s\nNoRTK runtime_root: %s\nNoRTK workspace2_root: %s\n' \
    "$repo/src" "$no_rtk_runtime" "$repo"
if [[ "$prepare_only" == true ]]; then
    exit 0
fi

set --
source /opt/ros/noetic/setup.bash
exec catkin_make -C "$workspace" --source "$repo/src" install -DROS_EDITION=ROS1 -DCMAKE_BUILD_TYPE=Release \
    -DPYTHON_EXECUTABLE=/usr/bin/python3 -DUGV_BRINGUP_GENERATED_CONFIG_DIR="$rtk_config" "${catkin_args[@]}"
