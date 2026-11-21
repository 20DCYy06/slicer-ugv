#!/usr/bin/env bash
set -eo pipefail

repo=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)

if [[ ${1:-} == --static ]]; then
    workspace=$(mktemp -d)
    trap 'rm -rf -- "$workspace"' EXIT
    bash "$repo/tools/build_native.sh" --prepare-only "$workspace"
    python3 - "$repo/src" "$workspace" <<'PY'
from pathlib import Path
import sys
from catkin_pkg.packages import find_packages
from catkin_pkg.topological_order import topological_order_packages
import yaml

sources, workspace = map(Path, sys.argv[1:])
packages = find_packages(str(sources), exclude_subspaces=True)
available = {package.name for package in packages.values()}
expected = {
    'fast_lio', 'livox_ros_driver2', 'slicer_bringup', 'slicer_navigation',
    'slicer_navigation_without_mapping', 'slicer_wasd_control', 'cup_gnss',
    'slicer_air_ground_msgs', 'ugv_bringup', 'ugv_navigation',
    'ugv_slicer_control', 'ugv_communication', 'ag_ugv_task', 'ag_uav_client',
    'slicer_visual_grasp',
}
assert available == expected, (expected - available, available - expected)
assert len(packages) == len(available)
assert all(path == package.name for path, package in packages.items())
assert all(path is not None for path, _package in topological_order_packages(packages))
runtime = workspace / 'runtime'
for source in ('no_rtk/.generated', 'rtk_config/online_runtime/.generated'):
    for filename in ('sensor_launch.yaml', 'fusion.yaml', 'lio.yaml', 'scan.yaml', 'MID360.json'):
        assert (runtime / source / filename).is_file(), (source, filename)
profile = yaml.safe_load((runtime / 'rtk_config/slicer_driver_body_speed.yaml').read_text())
assert profile['serial_number'] == '5CF0016377'
assert profile['serial_port'] == '/dev/c30d'
assert profile['pwm_signs'] == [-1, 1, -1, -1]
assert profile['encoder_signs'] == [-1, 1, 1, -1]
ports = dict(left_front='D', right_front='C', left_rear='A', right_rear='B')
assert profile['wheel_ports'] == ports
nortk = yaml.safe_load((runtime / 'no_rtk/config/control.yaml').read_text())
assert nortk['wheel_ports'] == ports
print('NATIVE_PREFLIGHT_OK: 15 packages, one message definition and both runtime profiles')
PY
    exit 0
fi

if [[ $# -ne 1 ]]; then
    echo "Usage: bash tools/check_native.sh --static | WORKSPACE" >&2
    exit 2
fi

workspace=$(realpath -- "$1")
source /opt/ros/noetic/setup.bash
source "$workspace/install/setup.bash"
for package in ugv_bringup slicer_bringup fast_lio livox_ros_driver2 ag_ugv_task \
    ag_uav_client cup_gnss ugv_communication slicer_visual_grasp; do
    [[ $(rospack find "$package") == "$workspace/install/share/$package" ]]
done
[[ -x "$workspace/install/lib/fast_lio/fastlio_mapping" ]]
[[ -x "$workspace/install/lib/livox_ros_driver2/livox_ros_driver2_node" ]]
[[ $(rosmsg md5 slicer_air_ground_msgs/Command) == b574bb9bb1240c0d2ae262d6c3feb2bf ]]
rosrun slicer_visual_grasp run.sh --check

roslaunch --nodes ugv_bringup slicer_field_sensors.launch
roslaunch --nodes ugv_bringup slicer_field_navigation.launch geographic:=false
roslaunch --nodes ugv_bringup slicer_field_navigation.launch
roslaunch --nodes ugv_bringup slicer_body_driver.launch
roslaunch --nodes ugv_bringup slicer_air_ground.launch
roslaunch --nodes slicer_navigation_without_mapping slicer_nav_without_mapping.launch \
    "runtime_root:=$workspace/runtime/no_rtk" "workspace2_root:=$repo" gui:=false
roslaunch --nodes ag_ugv_task ugv.launch start_fkie:=false monitor_only:=true
roslaunch --nodes ugv_communication fkie_ugv_sync.launch start_discovery:=true start_relay:=true
echo 'NATIVE_INSTALL_OK: packages, executable, message MD5 and launch graphs'
