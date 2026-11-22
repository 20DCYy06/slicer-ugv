#!/usr/bin/env bash
set -eo pipefail

repo=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)
static=false
if [[ ${1:-} == --static ]]; then
    static=true
    shift
fi
if [[ $# -gt 1 || ${1:-} == --help || ${1:-} == -h ]]; then
    echo "Usage: bash tools/check_native.sh [--static] [WORKSPACE]"
    exit 0
fi
workspace=$(realpath -m -- "${1:-${repo%/src/slicer-ugv}}")
if [[ "$repo" != "$workspace/src/slicer-ugv" ]]; then
    echo "Check out this repository at $workspace/src/slicer-ugv before checking." >&2
    exit 2
fi

python3 - "$repo/dependencies.repos" "$workspace" <<'PY_DEPS'
from pathlib import Path
import re
import subprocess
import sys

import yaml

manifest, workspace = map(Path, sys.argv[1:])
for relative, entry in yaml.safe_load(manifest.read_text())['repositories'].items():
    expected = entry['version']
    if entry['type'] != 'git' or not re.fullmatch(r'[0-9a-f]{40}', expected):
        sys.exit(f'Expected a full Git commit in dependencies.repos for {relative}.')
    try:
        actual = subprocess.check_output(
            ['git', '-C', str(workspace / relative), 'rev-parse', 'HEAD'], text=True).strip()
    except subprocess.CalledProcessError:
        sys.exit(f'Missing dependency checkout: {relative}')
    if actual != expected:
        sys.exit(f'{relative} is at {actual}, expected {expected}.')
PY_DEPS

packages=$(python3 - "$workspace/src" <<'PY'
from pathlib import Path
import sys

from catkin_pkg.packages import find_packages
from catkin_pkg.topological_order import topological_order_packages

packages = find_packages(sys.argv[1], exclude_subspaces=True)
external = {'third_party/fast_lio': 'fast_lio',
            'third_party/livox_ros_driver2': 'livox_ros_driver2'}
if any(path not in packages or packages[path].name != name for path, name in external.items()):
    sys.exit('The pinned FAST-LIO and Livox ROS packages must be present in src/third_party.')
for path, package in packages.items():
    if path not in external and Path(path).parts != ('slicer-ugv', 'src', package.name):
        sys.exit(f'Unexpected package location for {package.name}: {path}')

ordered = topological_order_packages(packages)
if any(path is None for path, _package in ordered):
    sys.exit('A cycle exists in the Catkin package dependency graph.')
if len(ordered) != len(packages):
    sys.exit('Some workspace packages are unavailable in the Catkin graph.')
print('\n'.join(sorted(pkg.name for pkg in packages.values())))
PY
)

if [[ "$static" == true ]]; then
    bash "$repo/tools/build_native.sh" --prepare-only "$workspace"
    hardware=$(realpath -m -- "${SLICER_HARDWARE_ROOT:-$workspace/deps/slicer_ugv_hardware}")
    python3 - "$workspace/runtime" "${SLICER_HARDWARE_CONFIG:-$hardware/config.yaml}" <<'PY'
from pathlib import Path
import sys

import yaml

runtime, hardware = map(Path, sys.argv[1:])

def load(path):
    return yaml.safe_load(path.read_text())

for profile in ('no_rtk/.generated', 'rtk_config/online_runtime/.generated'):
    for filename in ('sensor_launch.yaml', 'fusion.yaml', 'lio.yaml', 'scan.yaml', 'MID360.json'):
        if not (runtime / profile / filename).is_file():
            sys.exit(f'Missing generated profile: {profile}/{filename}')

vehicle = load(hardware)
host_copy = load(runtime / 'no_rtk/config/c30d_host.yaml')
driver = load(runtime / 'rtk_config/slicer_driver_body_speed.yaml')
serial = vehicle['serial']
expected = {'serial_port': serial['port'], 'serial_number': serial['serial_number'],
            'pwm_signs': serial['pwm_signs'], 'encoder_signs': serial['encoder_signs']}
if host_copy != vehicle or any(driver[key] != value for key, value in expected.items()):
    sys.exit('Generated NoRTK and RTK profiles disagree with the selected hardware configuration.')
PY
    printf 'NATIVE_PREFLIGHT_OK: Catkin package graph and both runtime profiles\n'
    exit 0
fi

source /opt/ros/noetic/setup.bash
source "$workspace/install/setup.bash"
export ROS_HOME="${ROS_HOME:-$workspace/logs/ros}"
export SLICER_RUNTIME_DIR="${SLICER_RUNTIME_DIR:-$workspace/runtime}"
while IFS= read -r package; do
    path=$(rospack find "$package")
    if [[ "$path" != "$workspace/install/share/$package" ]]; then
        printf 'Wrong overlay for %s: %s\n' "$package" "$path" >&2
        exit 1
    fi
done <<< "$packages"

[[ -x "$workspace/install/lib/fast_lio/fastlio_mapping" ]]
[[ -x "$workspace/install/lib/livox_ros_driver2/livox_ros_driver2_node" ]]
[[ -x "$workspace/install/lib/ugv_navigation/navigation_runtime.py" ]]
rosmsg show ugv_navigation/PrepareGeographicAction > /dev/null
[[ $(rosmsg md5 slicer_air_ground_msgs/Command) == b574bb9bb1240c0d2ae262d6c3feb2bf ]]

sdk="$workspace/sdk/install"
livox_build="$workspace/build/third_party/livox_ros_driver2/CMakeFiles/livox_ros_driver2_node.dir"
grep -F -- "$sdk/include" "$livox_build/flags.make" > /dev/null
grep -F -- "$sdk/lib/liblivox_lidar_sdk_static.a" "$livox_build/link.txt" > /dev/null

rosrun slicer_visual_grasp run.sh --check
roslaunch --nodes ugv_bringup slicer_field_sensors.launch
roslaunch --nodes ugv_bringup slicer_field_navigation.launch geographic:=false
roslaunch --nodes ugv_bringup slicer_field_navigation.launch geographic:=false preparation:=true
roslaunch --nodes ugv_bringup slicer_navigation_runtime.launch
roslaunch --nodes ugv_bringup slicer_field_navigation.launch
roslaunch --nodes ugv_bringup slicer_body_driver.launch
roslaunch --nodes ugv_bringup slicer_air_ground.launch
roslaunch --nodes slicer_navigation_without_mapping slicer_nav_without_mapping.launch \
    "runtime_root:=$workspace/runtime/no_rtk" gui:=false
roslaunch --nodes ag_ugv_task ugv.launch start_fkie:=false monitor_only:=true
roslaunch --nodes ugv_communication fkie_ugv_sync.launch start_discovery:=true start_relay:=true
echo 'NATIVE_INSTALL_OK: packages, SDK linkage, executable, message MD5 and launch graphs'
