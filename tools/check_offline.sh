#!/usr/bin/env bash
set -euo pipefail

repo=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
cd "$repo"
export PYTHONNOUSERSITE=1
export QT_QPA_PLATFORM=offscreen
# Load current source modules, even when the calling shell sourced an overlay.
export PYTHONPATH="$repo/src/ag_uav_client/src:$repo/src/ag_ugv_task/src:$repo/src/cup_gnss/src"

run_tests() {
    local directory=$1
    shift
    local pattern
    for pattern in "$@"; do
        printf 'Offline %s/%s\n' "$directory" "$pattern"
        /usr/bin/python3 -B -m unittest discover -s "$directory" -p "$pattern"
    done
}

# Discover only source-level suites. ROS message, transport and device tests
# live alongside these suites and have separate Noetic entry points.
run_tests src/ugv_navigation/test test_air_ground_cli.py test_air_ground_core.py \
    test_air_ground_executor.py test_air_ground_stop_boundaries.py 'test_geographic_*.py' \
    test_preparation_path.py
run_tests src/ag_uav_client/test test_protocol.py test_observation.py
run_tests src/ag_ugv_task/test test_route.py
run_tests src/ugv_slicer_control/test test_navigation_guard.py test_control.py \
    test_control_expected_hold.py
run_tests src/cup_gnss/tests test_enu.py
run_tests tools/rtk/desktop/tests test_dashboard.py test_workflow.py \
    test_workflow_discovery.py test_workflow_panel.py

run_tests tools/tests "test_*release.py"
