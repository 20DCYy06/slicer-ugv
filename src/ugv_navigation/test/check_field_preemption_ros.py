#!/usr/bin/env python3
"""Exercise GeographicClient and native Action preemption on an isolated master."""
import json
import subprocess
import sys
import time

import actionlib
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseActionGoal, MoveBaseResult
import rospy
from std_msgs.msg import String

from geographic import GeographicClient, arguments


ACTION = '/ugv/move_base'


def wait_until(condition, timeout=6.):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if condition():
            return
        time.sleep(.03)
    raise AssertionError('timed out waiting for isolated Action test')


def action_server():
    rospy.init_node('move_base')
    server = None

    def drive(goal):
        end = time.monotonic() + 3.
        while time.monotonic() < end and not rospy.is_shutdown():
            if server.is_preempt_requested():
                server.set_preempted(MoveBaseResult(), 'superseded by fixture target')
                return
            time.sleep(.02)
        server.set_succeeded(MoveBaseResult())

    server = actionlib.SimpleActionServer(ACTION, MoveBaseAction, execute_cb=drive, auto_start=False)
    server.start()
    rospy.spin()


def geographic_client(x, issued=None):
    rospy.init_node('slicer_geographic_client_fixture', anonymous=True)
    cli = ['send', '--east', str(x), '--north', '0', '--session-enu']
    if issued is not None:
        cli += ['--request-stamp', issued]
    args = arguments(cli)
    client = GeographicClient(args)
    wait_until(lambda: client.action_sample and client.mux_sample and client.status)
    try:
        result = client.run_action(client.pose([x, 0., 0.], 0.), 1)
    except ValueError as error:
        result = dict(reason=str(error))
    print(json.dumps(result), flush=True)


def test():
    rospy.init_node('slicer_velocity_mux')
    assert rospy.get_param('/rtk_field_test_isolated', False)
    geo = rospy.Publisher('/geographic/status', String, queue_size=1)
    mux = rospy.Publisher('/cup_nav/manual_state', String, queue_size=1)
    user_goal = rospy.Publisher(ACTION + '/goal', MoveBaseActionGoal, queue_size=1)
    actions = []
    sub = rospy.Subscriber(ACTION + '/goal', MoveBaseActionGoal,
                          lambda m: actions.append(m.goal.target_pose.pose.position.x))
    hold = False

    def publish(_event):
        stamp = rospy.Time.now().to_sec()
        geo.publish(String(data=json.dumps(dict(state='ready', registered=True, generation=1, status_stamp=stamp))))
        mux.publish(String(data=json.dumps(dict(manual_hold=hold, emergency_stop_latched=False,
                                               teleop_fresh=False, selected_source='nav'))))

    heartbeat = rospy.Timer(rospy.Duration(.05), publish)
    process = [subprocess.Popen([sys.executable, __file__, 'server', '__ns:=/ugv'])]
    try:
        def send(value, stamp=None):
            cmd = [sys.executable, __file__, 'client', str(value)]
            if stamp is not None:
                cmd.append(str(stamp))
            child = subprocess.Popen(cmd, stdout=subprocess.PIPE, text=True)
            process.append(child)
            return child

        def result(child):
            data, _ = child.communicate(timeout=8.)
            return json.loads(data.splitlines()[-1])

        # A first WGS84 request can wait through an entire registration run
        # before it reaches a newly started geographic Action server.
        first = send(1., rospy.Time.now().to_sec() - 45.)
        wait_until(lambda: 1. in actions)
        second = send(2.)
        wait_until(lambda: 2. in actions)
        assert result(first)['state'] == GoalStatus.PREEMPTED

        manual = MoveBaseActionGoal()
        manual.header.stamp = manual.goal_id.stamp = rospy.Time.now()
        manual.goal_id.id = 'slicer_rtk_desktop-field-test'
        manual.goal.target_pose.header.frame_id = 'map'
        manual.goal.target_pose.pose.position.x = 3.
        manual.goal.target_pose.pose.orientation.w = 1.
        user_goal.publish(manual)
        wait_until(lambda: 3. in actions)
        assert result(second)['state'] == GoalStatus.PREEMPTED

        older_stamp = manual.goal_id.stamp.to_sec()
        newest = send(4.)
        wait_until(lambda: 4. in actions)
        delayed = result(send(6., older_stamp))
        assert delayed['state'] == GoalStatus.RECALLED, delayed
        hold = True
        time.sleep(.15)
        assert 'Mux not available' in result(send(5.))['reason']
        assert 5. not in actions
        assert result(newest)['state'] == GoalStatus.SUCCEEDED
        heartbeat.shutdown()
        sub.unregister()
        print('PASS ordinary WGS84/RViz goals preempt on the Action server and manual HOLD rejects new goals')
    finally:
        for child in process:
            if child.poll() is None:
                child.terminate()
            try:
                child.wait(timeout=3.)
            except subprocess.TimeoutExpired:
                child.kill()
                child.wait()


if __name__ == '__main__':
    if len(sys.argv) > 1 and sys.argv[1] == 'server':
        action_server()
    elif len(sys.argv) > 2 and sys.argv[1] == 'client':
        geographic_client(float(sys.argv[2]), sys.argv[3] if len(sys.argv) > 3 else None)
    else:
        test()
