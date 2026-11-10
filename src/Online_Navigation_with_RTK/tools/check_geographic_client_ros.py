#!/usr/bin/env python3
"""TEST_ONLY geographic client transport check, no devices/external network.

From the built workspace root, use the existing isolated wrapper:
  UGV_NOETIC_IMAGE=ugv-online-offline:20261030 ./tools/noetic.sh \
    python3 tools/check_geographic_client_ros.py

Fake Action/GetPlan/registration sources test the client boundary only. These
do not establish actual alignment, localization accuracy or physical motion.
"""
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import threading
import time
import xmlrpc.client


# Observe the same snapshots consumed by the real client without changing its
# freshness checks. Each subprocess emits one concise diagnostic even on failure.
CLIENT_WITH_STATUS_AGES = r'''
import inspect
import json
import math
import runpy
import sys
import time

client_path = sys.argv.pop(1)
sys.argv[0] = client_path
api = runpy.run_path(client_path, run_name='TEST_ONLY_status_age_probe')
client_type = api['GeographicClient']
original_observation = client_type.observation
original_status = client_type._status
evidence = {}

def record(client, phase, observation):
    status, received, _ = observation
    before_mono = time.monotonic()
    now_ros, now_wall, now_mono = client.ros.Time.now().to_sec(), time.time(), time.monotonic()
    stamp = status.get('status_stamp') if isinstance(status, dict) else None
    valid_stamp = isinstance(stamp, (float, int)) and math.isfinite(stamp)
    sample = dict(receive_age_s=None if received is None else now_mono-received,
                  stamp_age_s=now_ros-stamp if valid_stamp else None,
                  ros_now=now_ros, wall_now=now_wall, monotonic_now=now_mono,
                  sample_duration_s=now_mono-before_mono,
                  status_stamp=stamp, generation=status.get('generation') if isinstance(status, dict) else None)
    row = evidence.setdefault(phase, dict(samples=0, status_max_age_s=client.status_age,
                                         first=sample, last=sample))
    if row['samples']:
        previous = row['last']
        sample['ros_elapsed_s'] = now_ros-previous['ros_now']
        sample['wall_elapsed_s'] = now_wall-previous['wall_now']
        sample['monotonic_elapsed_s'] = now_mono-previous['monotonic_now']
        drift = max(abs(sample[key]-sample['monotonic_elapsed_s'])
                    for key in ('ros_elapsed_s', 'wall_elapsed_s'))
        row['max_clock_delta_s'] = max(drift, row.get('max_clock_delta_s', 0.))
        if drift > .1:
            row.setdefault('clock_steps', []).append(sample)
    row['samples'] += 1
    row['last'] = sample
    for key in ('receive_age_s', 'stamp_age_s'):
        value = sample[key]
        if value is not None:
            row['min_'+key] = min(value, row.get('min_'+key, value))
            row['max_'+key] = max(value, row.get('max_'+key, value))

def observation(client):
    value = original_observation(client)
    record(client, inspect.currentframe().f_back.f_code.co_name, value)
    return value

def status_callback(client, message):
    original_status(client, message)
    record(client, 'status_callback', original_observation(client))

client_type.observation = observation
client_type._status = status_callback
try:
    sys.exit(api['main']())
finally:
    print('TEST_ONLY_STATUS_AGES '+json.dumps(evidence, sort_keys=True), file=sys.stderr)
'''

assert Path('/.dockerenv').exists()
assert {path.name for path in Path('/sys/class/net').iterdir()} == {'lo'}
workspace = Path(__file__).resolve().parents[1]
os.environ.update(ROS_MASTER_URI='http://127.0.0.1:11311', ROS_IP='127.0.0.1')
master = subprocess.Popen(['roscore'], stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT, start_new_session=True)
thread = None
stop = threading.Event()
publisher_clock = dict(samples=0)
try:
    deadline = time.monotonic()+10.
    while time.monotonic() < deadline:
        try:
            xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI']).getPid('/TEST_ONLY_client_check')
            break
        except OSError:
            time.sleep(.1)
    import actionlib
    import rospy
    import tf2_ros
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
    from geometry_msgs.msg import TransformStamped
    from move_base_msgs.msg import MoveBaseAction, MoveBaseResult
    from nav_msgs.msg import Odometry, Path as RosPath
    from nav_msgs.srv import GetPlan, GetPlanResponse
    from sensor_msgs.msg import NavSatFix
    from std_msgs.msg import String
    from std_srvs.srv import Trigger, TriggerResponse
    sys.path.insert(0, str(workspace/'src/ugv_navigation/scripts'))
    from geographic_core import enu_to_geodetic
    rospy.init_node('TEST_ONLY_geographic_sources', disable_signals=True)
    rospy.set_param('/ugv/move_base/global_costmap/global_frame', 'map')
    rospy.set_param('/geographic/stop_window_s', 1.)
    status = dict(state='ready', registered=True, generation=7, datum='WGS84',
                  antenna_in_base_link_xy_m=[.4, -.2],
                  origin=dict(latitude=0., longitude=0., altitude_ellipsoid_m=0.))
    # At yaw=pi/2 the lever [.4,-.2] maps to ENU [.2,.4].
    antenna_geodetic = enu_to_geodetic([2.2, 3.4, 0.], status['origin'])
    state = dict(mode='success', goals=[], preempts=0, plans=0, begins=0, finishes=0)
    received_goals, received_results = [], []
    status_pub = rospy.Publisher('/geographic/status', String, queue_size=1, latch=True)
    odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
    global_pub = rospy.Publisher('/global/odom', Odometry, queue_size=10)
    fix_pub = rospy.Publisher('/gnss/fix', NavSatFix, queue_size=10)
    quality_pub = rospy.Publisher('/gnss/fix_quality', DiagnosticArray, queue_size=10)
    rospy.Subscriber('/geographic/goal', String, lambda message: received_goals.append(json.loads(message.data)))
    rospy.Subscriber('/geographic/goal_result', String, lambda message: received_results.append(json.loads(message.data)))
    broadcaster = tf2_ros.TransformBroadcaster()

    def publish():
        while not stop.is_set():
            before_mono = time.monotonic()
            now = rospy.Time.now()
            clock = dict(ros_now=now.to_sec(), wall_now=time.time(), monotonic_now=time.monotonic())
            clock['sample_duration_s'] = clock['monotonic_now']-before_mono
            if publisher_clock['samples']:
                previous = publisher_clock['last']
                for source in ('ros', 'wall', 'monotonic'):
                    clock[source+'_elapsed_s'] = clock[source+'_now']-previous[source+'_now']
                drift = max(abs(clock[key]-clock['monotonic_elapsed_s'])
                            for key in ('ros_elapsed_s', 'wall_elapsed_s'))
                publisher_clock['max_clock_delta_s'] = max(drift, publisher_clock.get('max_clock_delta_s', 0.))
                if drift > .1:
                    publisher_clock.setdefault('clock_steps', []).append(clock)
            else:
                publisher_clock['first'] = clock
            publisher_clock['samples'] += 1
            publisher_clock['last'] = clock
            snapshot = dict(status, status_stamp=now.to_sec(), registration_samples=2,
                            qualified_stamp=now.to_sec(), latest_observation=dict(
                stamp=now.to_sec(), base_xy=[2., 3.], gga_quality=4,
                fresh_rtk_usable=True, covariance_assumed=True))
            status_pub.publish(String(data=json.dumps(snapshot)))
            odom = Odometry()
            odom.header.stamp, odom.header.frame_id, odom.child_frame_id = now, 'odom', 'base_link'
            odom.pose.pose.position.x, odom.pose.pose.position.y = 12., -7.
            odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = math.sin(math.pi/4.), math.cos(math.pi/4.)
            odom_pub.publish(odom)
            global_odom = Odometry()
            global_odom.header.stamp, global_odom.header.frame_id, global_odom.child_frame_id = now, 'map', 'base_link'
            global_odom.pose.pose.position.x, global_odom.pose.pose.position.y = 2., 3.
            global_odom.pose.pose.orientation = odom.pose.pose.orientation
            global_pub.publish(global_odom)
            fix = NavSatFix()
            fix.header.stamp, fix.header.frame_id, fix.status.status = now, 'gnss_link', 2
            fix.latitude, fix.longitude, fix.altitude = antenna_geodetic
            fix_pub.publish(fix)
            quality = DiagnosticArray()
            quality.header.stamp = now
            quality.status = [DiagnosticStatus(name='cup_gnss/receiver', values=[
                KeyValue(key=key, value=value) for key, value in (
                    ('gga_quality', '4'), ('fresh_rtk_usable', 'true'),
                    ('fix_valid', 'true'), ('session', 'TEST_ONLY_client_probe'))])]
            quality_pub.publish(quality)
            transforms = []
            for parent, child, x, y, yaw in [('map', 'odom', 0., 0., 0.), ('odom', 'base_link', 12., -7., math.pi/2.)]:
                tf = TransformStamped()
                tf.header.stamp, tf.header.frame_id, tf.child_frame_id = now, parent, child
                tf.transform.translation.x, tf.transform.translation.y = x, y
                tf.transform.rotation.z, tf.transform.rotation.w = math.sin(yaw/2.), math.cos(yaw/2.)
                transforms.append(tf)
            broadcaster.sendTransform(transforms)
            stop.wait(.05)

    def execute(goal):
        state['goals'].append(goal)
        if state['mode'] == 'cancel':
            time.sleep(.2)
            status['generation'] += 1
            until = time.monotonic()+5.
            while time.monotonic() < until:
                if server.is_preempt_requested():
                    state['preempts'] += 1
                    server.set_preempted(MoveBaseResult(), 'TEST_ONLY cancellation received')
                    return
                time.sleep(.02)
            server.set_aborted(MoveBaseResult(), 'TEST_ONLY no cancel received')
        else:
            time.sleep(.2)
            server.set_succeeded(MoveBaseResult(), 'TEST_ONLY arrived')

    def plan(request):
        state['plans'] += 1
        return GetPlanResponse(plan=RosPath(header=request.goal.header, poses=[request.start, request.goal]))

    def begin(request):
        state['begins'] += 1
        status.update(state='collecting', registered=False, generation=status['generation']+1)
        return TriggerResponse(success=True, message=json.dumps({'reason': '', 'generation': status['generation']}))

    def finish(request):
        state['finishes'] += 1
        status.update(state='ready', registered=True)
        return TriggerResponse(success=True, message=json.dumps({'displacement_m': 2.5, 'rmse_m': .03}))

    server = actionlib.SimpleActionServer('/ugv/move_base', MoveBaseAction, execute_cb=execute, auto_start=False)
    server.start()
    services = [rospy.Service('/ugv/move_base/make_plan', GetPlan, plan),
                rospy.Service('/geographic/begin_registration', Trigger, begin),
                rospy.Service('/geographic/finish_registration', Trigger, finish)]
    thread = threading.Thread(target=publish)
    thread.start()

    status_age_evidence = []

    def run(arguments, wanted_code):
        process = subprocess.run([sys.executable, '-c', CLIENT_WITH_STATUS_AGES,
                                  str(workspace/'src/ugv_navigation/scripts/geographic.py')]+arguments,
                                 text=True, capture_output=True, timeout=25.)
        prefix = 'TEST_ONLY_STATUS_AGES '
        age_lines = [line[len(prefix):] for line in process.stderr.splitlines() if line.startswith(prefix)]
        diagnostic = dict(arguments=arguments, returncode=process.returncode,
                          status_ages=json.loads(age_lines[-1]) if age_lines else None)
        if process.returncode != wanted_code or not process.stdout.strip():
            diagnostic.update(stdout=process.stdout, stderr='\n'.join(
                line for line in process.stderr.splitlines() if not line.startswith(prefix)))
        status_age_evidence.append(diagnostic)
        print('TEST_ONLY_CLIENT_CHECK '+json.dumps(diagnostic, sort_keys=True), file=sys.stderr, flush=True)
        assert process.returncode == wanted_code, (process.returncode, process.stdout, process.stderr)
        assert diagnostic['status_ages'], 'client emitted no status-age evidence'
        assert process.stdout.strip(), ('client returned no result', process.stderr)
        return json.loads(process.stdout)

    plan_result = run(['plan', '--east', '2', '--north', '3', '--session-enu'], 0)
    assert plan_result['poses'] == 2 and not plan_result['action_goal_sent']
    assert state['plans'] == 1 and not state['goals']
    arrived = run(['send', '--east', '2', '--north', '3', '--session-enu', '--timeout', '5'], 0)
    assert arrived['geographic']['confirmed'] and arrived['stop']['confirmed']
    assert arrived['geographic']['horizontal_residual_m'] == 0.
    assert len(state['goals']) == 1 and len(received_results) == 2
    state['mode'] = 'cancel'
    invalidated = run(['send', '--east', '2', '--north', '3', '--session-enu', '--timeout', '5'], 1)
    assert state['preempts'] == 1 and invalidated['move_base']['state'] == 2
    assert 'generation' in invalidated['move_base']['reason']
    assert not invalidated['geographic']['confirmed']
    state['mode'] = 'success'
    rospy.set_param('/ugv/move_base/global_costmap/global_frame', 'odom')
    initialized = run(['initialize', '--distance', '2.5', '--timeout', '5'], 0)
    assert state['begins'] == state['finishes'] == 1
    assert initialized['registration']['success']
    pose = state['goals'][-1].target_pose
    assert pose.header.frame_id == 'odom'
    assert abs(pose.pose.position.x-12.) < 1e-6 and abs(pose.pose.position.y+4.5) < 1e-6
    assert initialized['stop']['observed_quiet_s'] >= 2.
    rospy.set_param('/ugv/move_base/global_costmap/global_frame', 'map')
    with tempfile.TemporaryDirectory(prefix='TEST_ONLY_geographic_capture_') as folder:
        target_file = Path(folder)/'target.json'
        before_capture = (len(state['goals']), state['plans'], state['begins'], state['finishes'], len(received_goals))
        captured = run(['capture', '--duration', '.3', '--output', str(target_file)], 0)
        assert before_capture == (len(state['goals']), state['plans'], state['begins'], state['finishes'], len(received_goals))
        assert json.loads(target_file.read_text()) == captured
        assert captured['kind'] == 'slicer_geographic_target' and captured['version'] == 1
        assert captured['physical_reference'] == 'base_link_horizontal'
        assert captured['capture']['sample_count'] >= 2
        assert captured['capture']['stationary_window']['confirmed']
        assert captured['capture']['quality']['gga_quality'] == 4
        assert all(abs(a-b) < 1e-6 for a, b in zip(captured['capture']['mean_base_enu_m'][:2], [2., 3.]))
        # A new registration generation must not invalidate the WGS84 file.
        status['generation'] += 1
        file_plan = run(['plan', '--target-file', str(target_file)], 0)
        assert file_plan['poses'] == 2 and not file_plan['action_goal_sent']
        assert len(state['goals']) == before_capture[0] and state['plans'] == before_capture[1]+1
        assert all(abs(a-b) < 1e-6 for a, b in zip(file_plan['target']['map_xyz'][:2], [2., 3.]))
        file_arrived = run(['send', '--target-file', str(target_file), '--timeout', '5'], 0)
        assert file_arrived['move_base']['state'] == 3 and file_arrived['geographic']['confirmed']
        assert file_arrived['stop']['confirmed'] and file_arrived['geographic']['horizontal_residual_m'] < 1e-6
        assert len(state['goals']) == before_capture[0]+1
        assert state['goals'][-1].target_pose.header.frame_id == 'map'
    print(json.dumps(dict(TEST_ONLY=True, no_devices=True, network='none', passed=True,
                         plan_only=plan_result, arrival=arrived, invalidated=invalidated,
                         initialize=initialized, capture=captured, target_file_plan=file_plan,
                         target_file_arrival=file_arrived, status_age_evidence=status_age_evidence), indent=2))
finally:
    stop.set()
    if thread:
        thread.join(timeout=3.)
    print('TEST_ONLY_PUBLISHER_CLOCK '+json.dumps(publisher_clock, sort_keys=True), file=sys.stderr, flush=True)
    try:
        rospy.signal_shutdown('TEST_ONLY done')
    except NameError:
        pass
    os.killpg(master.pid, signal.SIGINT)
    master.wait(timeout=10.)
