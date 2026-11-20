#!/usr/bin/env python3
"""Send one captured real WGS84 target, then observe asynchronous state.

Usage: rosrun ugv_navigation air_ground_rospy.py /path/to/captured_target.json
Run this as the only logical producer. No receipt/aircraft ACK gates execution.
"""
import argparse
import json
from pathlib import Path
import sys
import time
import uuid

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from air_ground_wire import (COMMAND_TOPIC, PROTOCOL_VERSION, STATE_TOPIC, load_target,
                             make_command, state_problem)


def main():
    import rospy
    from slicer_air_ground_msgs.msg import Command, GeoPoint
    from std_msgs.msg import String

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('target_file', type=Path)
    args = parser.parse_args(rospy.myargv()[1:])
    # File validation happens before any command is sent; no artificial coordinates.
    target = load_target(args.target_file)
    rospy.init_node('air_ground_example', anonymous=True)
    publisher = rospy.Publisher(COMMAND_TOPIC, Command, queue_size=1, latch=False)
    deadline = time.monotonic()+5.
    while not rospy.is_shutdown():
        left = deadline-time.monotonic()
        if left <= 0.:
            raise RuntimeError('no fresh air-ground state/command subscriber')
        message = rospy.wait_for_message(STATE_TOPIC, String, timeout=left)
        state = json.loads(message.data)
        now = time.monotonic()
        if state_problem(state, now, now, rospy.Time.now().to_sec()) is None and publisher.get_num_connections():
            break
        time.sleep(.02)
    else:
        return

    command = dict(protocol_version=PROTOCOL_VERSION, server_id=state['server_id'],
                   sequence=(state.get('last_sequence') or 0)+1,
                   request_id=str(uuid.uuid4()), task_id=str(uuid.uuid4()), operation='SET_TASK',
                   final_goal=target, via_points=[])
    # Subscribe before publishing so fast state transitions are observable too.
    subscriber = rospy.Subscriber(STATE_TOPIC, String, lambda message: print(message.data, flush=True), queue_size=1)
    publisher.publish(make_command(command, Command, GeoPoint))
    print('Published SET_TASK request_id=' + command['request_id'] + '; observe state for acceptance and final result.', flush=True)
    rospy.spin()


if __name__ == '__main__':
    main()
