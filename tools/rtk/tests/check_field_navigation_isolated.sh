#!/usr/bin/env bash
set -eo pipefail
repo=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd -P)
workspace=$(realpath -m -- "${1:-${repo%/src/slicer-ugv}}")
source "$workspace/install/setup.bash"
export PYTHONPATH="$workspace/install/lib/ugv_navigation${PYTHONPATH:+:$PYTHONPATH}"
root=$(mktemp -d /tmp/slicer-field-check.XXXXXXXX)
export ROS_HOME="$root" ROS_IP=127.0.0.1
unset ROS_HOSTNAME ROS_NAMESPACE
if [[ $(rospack find slicer_navigation) != "$workspace/install/share/slicer_navigation" ]]; then
    printf 'Unexpected DWA overlay\n' >&2
    exit 2
fi
port=$(python3 -c 'import socket; s=socket.socket(); s.bind(("127.0.0.1",0)); print(s.getsockname()[1])')
export ROS_MASTER_URI="http://127.0.0.1:$port"
roscore -p "$port" >"$root/master.log" 2>&1 &
core=$!
field=
cleanup() {
    if [[ -n "$field" ]]; then
        kill -INT "$field" 2>/dev/null || true
        wait "$field" 2>/dev/null || true
    fi
    kill -TERM "$core" 2>/dev/null || true
    wait "$core" 2>/dev/null || true
    printf 'ROS evidence %s\n' "$root"
}
trap cleanup EXIT
for _ in {1..30}; do
    if rosparam set /rtk_field_test_isolated true >/dev/null 2>&1; then break; fi
    sleep .2
done
[[ $(rosparam get /rtk_field_test_isolated) = true ]]
"$workspace/install/lib/slicer_navigation/slicer_field_commitment_check"
timeout --kill-after=3 40 "$workspace/install/lib/slicer_navigation/slicer_field_navigation_ros_check"
timeout --kill-after=3 40 python3 -B "$repo/src/ugv_navigation/test/check_field_preemption_ros.py"

# Exercise the actual installed launch and its resolved configuration. The
# isolated master receives only synthetic TF, with no sensors, driver or goal.
roslaunch --screen --sigint-timeout 4 --sigterm-timeout 2 ugv_bringup \
    slicer_field_navigation.launch geographic:=true >"$root/field-map.log" 2>&1 &
field=$!
if ! timeout --kill-after=2 28 python3 -B - <<'PY_ROS'
import json

import rospy
import tf
from actionlib_msgs.msg import GoalStatusArray
from geometry_msgs.msg import Twist
from std_msgs.msg import String

rospy.init_node('check_field_map_startup')
broadcaster = tf.TransformBroadcaster()


def pose(_event):
    now = rospy.Time.now()
    broadcaster.sendTransform((0, 0, 0), (0, 0, 0, 1), now, 'odom', 'map')
    broadcaster.sendTransform((0, 0, 0), (0, 0, 0, 1), now, 'base_link', 'odom')


timer = rospy.Timer(rospy.Duration(.05), pose)
guard = rospy.wait_for_message('/slicer_nav_arc_guard/hold_reason', String, 10.)
assert guard._connection_header['callerid'] == '/slicer_nav_arc_guard'
assert guard.data == 'command missing/stale', guard
assert rospy.get_param('/slicer_nav_arc_guard/effective_max_reverse') == 0.

mux = rospy.wait_for_message('/cup_nav/manual_state', String, 5.)
assert mux._connection_header['callerid'] == '/slicer_velocity_mux'
state = json.loads(mux.data)
assert state['manual_hold'] and state['hold_reason'] == 'startup', state
command = rospy.wait_for_message('/cmd_vel', Twist, 5.)
assert command._connection_header['callerid'] == '/slicer_velocity_mux'
assert command == Twist(), command

status = rospy.wait_for_message('/ugv/move_base/status', GoalStatusArray, 5.)
assert status._connection_header['callerid'] == '/ugv/move_base'
assert not status.status_list, status
timer.shutdown()
print('PASS installed geographic launch: Guard running, Mux in startup HOLD, move_base idle')
PY_ROS
then
    rg -n 'Traceback|Error|REQUIRED process|failed' "$root/field-map.log" >&2 || true
    exit 1
fi
