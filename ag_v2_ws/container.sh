#!/usr/bin/env bash
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
rtk="$root/../src/Online_Navigation_with_RTK"
nortk="$root/../src/Online_Navigation_Without_RTK"
image=workspace2-communication:local
case "${1:-}" in
  build)
    name=ugv-ag-v2-build
    network=none
    shift
    set -- bash -c 'set -e; cd /ag_v2_ws
      catkin_make --only-pkg-with-deps ag_ugv_task -DPYTHON_EXECUTABLE=/usr/bin/python3 -DCATKIN_ENABLE_TESTING=OFF -j2
      catkin_make install -j2'
    ;;
  verify)
    name=ugv-ag-v2-verify
    network=none
    shift
    set -- bash -c 'set -e
      source /ag_v2_ws/install/setup.bash
      for pkg in ag_ugv_task ag_uav_client ugv_navigation ugv_slicer_control fkie_master_discovery fkie_master_sync; do rospack find "$pkg"; done
      roslaunch --nodes ag_ugv_task ugv.launch
      python3 -c "from ag_ugv_task.node import load_vehicle_modules; from ag_ugv_task.executor import TaskExecutor; import actionlib, move_base_msgs.msg, tf2_ros; print([m.__file__ for m in load_vehicle_modules()]); print(TaskExecutor.__name__)"'
    ;;
  radio-test)
    name=ugv-ag-v2-radio-test
    network=host
    export ROS_MASTER_URI=http://192.168.10.33:21311 ROS_IP=192.168.10.33
    shift
    set -- bash -c 'set -e; source /ag_v2_ws/install/setup.bash; exec python3 /ag_v2_ws/radio-test.py'
    ;;
  start)
    name=ugv-ag-v2
    network=host
    export ROS_MASTER_URI="${ROS_MASTER_URI:-http://192.168.10.33:11311}"
    export ROS_IP="${ROS_IP:-192.168.10.33}"
    shift
    set -- bash -c 'set -e; source /ag_v2_ws/install/setup.bash; exec roslaunch ag_ugv_task ugv.launch "$@"' bash "$@"
    ;;
  *)
    echo 'Usage: container.sh build|verify|start [launch args]'
    exit 2
    ;;
esac
exec docker run --rm --name "$name" --network "$network" --cap-drop ALL \
  --security-opt no-new-privileges --user "$(id -u):$(id -g)" \
  --mount "type=bind,src=$(realpath "$rtk"),dst=/workspace,readonly" \
  --mount "type=bind,src=$(realpath "$nortk"),dst=/slicer,readonly" \
  --mount "type=bind,src=$root,dst=/ag_v2_ws" --workdir /ag_v2_ws \
  -e HOME=/ag_v2_ws -e ROS_HOME=/ag_v2_ws/.ros -e ROS_IP -e ROS_MASTER_URI \
  -e PYTHONDONTWRITEBYTECODE=1 --entrypoint /bin/bash "$image" -c '
    set -eo pipefail
    run_args=("$@"); set --
    unset ROS_HOSTNAME
    source /opt/ros/noetic/setup.bash
    source /slicer/catkin_ws/devel/setup.bash
    source /workspace/install/setup.bash --extend
    exec "${run_args[@]}"
  ' bash "$@"
