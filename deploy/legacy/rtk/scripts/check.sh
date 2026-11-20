#!/usr/bin/env bash
set -euo pipefail
cd /workspace
source devel/setup.bash
export PYTHONDONTWRITEBYTECODE=1
# 由catkin/rostest分别启动ROS传输用例的私有Master，不把它们当普通unittest运行。
catkin_make run_tests -j1 -l2
catkin_test_results --all build/test_results
source install/setup.bash
python3 tools/test_workspace2_migration.py
# 原源码集成探针自行加入/workspace/src；使用devel避免与install双重可执行项。
source devel/setup.bash
python3 tools/check_geographic_localization_ros.py /workspace/logs/global-integration
python3 tools/check_geographic_client_ros.py /workspace/logs/client-integration
python3 tools/probe_outdoor_costmap.py --output /workspace/logs/costmap-integration.json
printf 'WORKSPACE2_RTK_CHECK_PASS\n'
