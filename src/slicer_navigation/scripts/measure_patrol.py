#!/usr/bin/env python3
"""Read-only patrol timing/velocity recorder. Never sends goals or motor commands."""
import argparse
import json
import math
from pathlib import Path
import threading
import time
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True)
    parser.add_argument('--duration', type=float, default=180.)
    args = parser.parse_args()
    rospy.init_node('slicer_patrol_measurement', anonymous=True)
    lock = threading.Lock()
    state = dict(stage='WAITING', point=None, speeds={}, started={}, results={}, done=False)

    def speed(msg, name):
        twist = msg.twist.twist if isinstance(msg, Odometry) else msg
        with lock:
            state['speeds'][name] = (time.monotonic(), twist.linear.x, twist.angular.z)

    def stage(msg):
        now, text = time.monotonic(), msg.data
        with lock:
            state['stage'] = text
            if text.startswith('MOVING '):
                name = next(field[6:] for field in text.split() if field.startswith('point='))
                state['point'] = name
                state['started'][name] = now
                state['results'][name] = dict(samples=0, reverse_samples=0, turning_samples=0,
                                              forward_samples=0, stationary_samples=0)
            elif text.startswith('REACHED '):
                name = text.split()[1]
                result = state['results'].setdefault(name, {})
                elapsed = now-state['started'][name] if name in state['started'] else None
                for field in text.split():
                    if field.startswith('elapsed='):
                        elapsed = float(field[8:].rstrip('s'))
                result['elapsed_s'] = elapsed
                print('MEASURE_REACHED', name, 'elapsed_s=', elapsed, flush=True)
                state['point'] = None
            elif text == 'COMPLETED' or text.startswith('FAILED:'):
                state['done'] = True

    for name, topic, message in [('raw', '/cmd_vel/nav_raw', Twist),
                                 ('smooth', '/cmd_vel/nav', Twist),
                                 ('motor_command', '/cmd_vel', Twist),
                                 ('wheel', '/wheel/odom', Odometry)]:
        rospy.Subscriber(topic, message, speed, callback_args=name, queue_size=1)
    rospy.Subscriber('/slicer_multi_goal_patrol/state', String, stage, queue_size=20)
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    started, last_print = time.monotonic(), -math.inf
    with output.open('w', encoding='utf-8') as stream:
        stream.write(json.dumps(dict(type='parameters', teb=rospy.get_param('/move_base/TebLocalPlannerROS', {})))+'\n')
        print('MEASURE_READY read-only', str(output), flush=True)
        while not rospy.is_shutdown() and time.monotonic()-started < args.duration:
            now = time.monotonic()
            with lock:
                speeds = {name: dict(age_s=round(now-value[0], 3), vx=value[1], wz=value[2])
                          for name,value in state['speeds'].items()}
                point = state['point']
                if point is not None:
                    result = state['results'][point]
                    command = speeds.get('motor_command')
                    if command and command['age_s'] <= .5:
                        v, w = command['vx'], command['wz']
                        result['samples'] += 1
                        result['reverse_samples'] += int(v < -.001)
                        result['forward_samples'] += int(v > .001)
                        result['turning_samples'] += int(abs(w) > .1)
                        result['stationary_samples'] += int(abs(v) < .001 and abs(w) < .01)
                row = dict(type='sample', elapsed_s=now-started, stage=state['stage'], speeds=speeds)
                done = state['done']
            stream.write(json.dumps(row)+'\n')
            stream.flush()
            if now-last_print >= 5.:
                print('MEASURE', row['stage'], json.dumps(speeds), flush=True)
                last_print = now
            if done:
                break
            time.sleep(.1)
    with lock:
        summary = dict(stage=state['stage'], results=state['results'])
    output.with_suffix('.summary.json').write_text(json.dumps(summary, indent=2)+'\n', encoding='utf-8')
    print('MEASURE_SUMMARY', json.dumps(summary), flush=True)


if __name__ == '__main__':
    main()
