#!/usr/bin/env bash
set -e

if [[ ${1:-} != ROS1 ]]; then
    echo 'Usage: ./build.sh ROS1 [catkin_make options]' >&2
    echo 'Build the ROS2 version from a separate upstream Livox workspace.' >&2
    exit 2
fi
shift
checkout=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
exec catkin_make -C "$checkout" -DROS_EDITION=ROS1 "$@"
