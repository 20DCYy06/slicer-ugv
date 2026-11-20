#!/usr/bin/env python3
"""TEST_ONLY real geographic adapter + robot_localization in an isolated master.

Run in a no-device Noetic container with --network none.
All odometry, GNSS and local TF are deterministic software inputs. The existing
Mux stays held and emits zero Twist only on /TEST_ONLY_chain/final. A reset
invokes the production cancel/hold path and checks its actual Mux acknowledgement.
No release, nonzero velocity command, driver, device or external network is used;
without a driver and MCU this cannot verify physical stop.
"""
from collections import deque
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
import traceback
import xmlrpc.client


def main():
    if (not Path('/.dockerenv').exists() or
            {item.name for item in Path('/sys/class/net').iterdir()} != {'lo'}):
        raise RuntimeError('TEST_ONLY requires a container with no devices and --network none')
    workspace = Path(__file__).resolve().parents[2]
    output = Path(sys.argv[1]) if len(sys.argv) > 1 else workspace / '.test-results/outdoor-rtk-20261031'
    output.mkdir(parents=True, exist_ok=True)
    report_path = output / 'global-probe.json'
    log_path = output / 'global-probe.log'
    os.environ.update(ROS_MASTER_URI='http://127.0.0.1:12427', ROS_IP='127.0.0.1')
    os.environ.pop('ROS_HOSTNAME', None)
    # Keep only the sourced Catkin environment; no injected script search path
    # that could hide broken devel relay imports.
    os.environ['ROS_PACKAGE_PATH'] = str(workspace/'src') + ':' + os.environ.get('ROS_PACKAGE_PATH', '')
    processes, subscribers, thread = [], [], None
    stop, lock = threading.Event(), threading.RLock()
    result = dict(TEST_ONLY=True, no_devices=True, network='none', passed=False,
                  estimator='actual Noetic robot_localization/ekf_localization_node',
                  physical_stop_verified=False,
                  hold_scope='Actual reset-to-held-Mux IPC acknowledgement; no driver/MCU.',
                  synthetic_accuracy_is_not_vehicle_accuracy=True)
    started, overall_deadline = time.monotonic(), time.monotonic()+80.
    log = log_path.open('w')
    temporary = tempfile.TemporaryDirectory(prefix='ugv-geographic-probe-')

    def launch(command):
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                   env=os.environ.copy(), start_new_session=True)
        processes.append(process)
        return process

    def wait_for(predicate, explanation, timeout=6.):
        deadline = min(time.monotonic()+timeout, overall_deadline)
        while time.monotonic() < deadline:
            if predicate():
                return
            if any(process.poll() is not None for process in processes):
                raise AssertionError('Child exited while waiting for '+explanation)
            time.sleep(.05)
        raise AssertionError('Timed out: '+explanation)

    def phase(duration):
        deadline = min(time.monotonic()+duration, overall_deadline)
        while time.monotonic() < deadline:
            if any(process.poll() is not None for process in processes):
                raise AssertionError('Child exited during synthetic phase')
            time.sleep(.05)
        if time.monotonic() >= overall_deadline:
            raise AssertionError('80 second probe deadline exceeded')

    try:
        launch(['roscore', '-p', '12427'])
        master = xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI'])

        def master_ready():
            try:
                return master.getPid('/TEST_ONLY_geographic_probe')[0] == 1
            except (OSError, xmlrpc.client.Error):
                return False

        wait_for(master_ready, 'private ROS master', 10.)
        import rospy
        import tf2_ros
        import yaml
        from actionlib_msgs.msg import GoalID
        from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
        from geometry_msgs.msg import TransformStamped, Twist
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import NavSatFix
        from std_msgs.msg import String
        from std_srvs.srv import Trigger
        from tf2_msgs.msg import TFMessage

        rospy.init_node('TEST_ONLY_geographic_probe', disable_signals=True)
        state = dict(distance=0., moving_since=None, velocity=0., shift=0., quality=4,
                     fixes=True, stale=False, reset=False)
        received = dict(status=None, globals=deque(maxlen=2000), local_tf=None,
                        map_tf=None, cancel=0, observations=0, tf_owners={},
                        manual_state=None, manual_state_caller=None,
                        mux_outputs=0, mux_nonzero_outputs=0, mux_unheld_states=0)

        def status_callback(message):
            with lock:
                received['status'] = json.loads(message.data)

        def global_callback(message):
            with lock:
                received['globals'].append(dict(stamp=message.header.stamp.to_sec(),
                                                x=message.pose.pose.position.x,
                                                y=message.pose.pose.position.y,
                                                caller=message._connection_header.get('callerid')))

        def cancel_callback(_message):
            with lock:
                received['cancel'] += 1

        def observation_callback(_message):
            with lock:
                received['observations'] += 1

        def manual_state_callback(message):
            manual = json.loads(message.data)
            with lock:
                received['manual_state'] = manual
                received['manual_state_caller'] = message._connection_header.get('callerid')
                if manual.get('manual_hold') is not True:
                    received['mux_unheld_states'] += 1

        def mux_output_callback(message):
            vector = (message.linear.x, message.linear.y, message.linear.z,
                      message.angular.x, message.angular.y, message.angular.z)
            with lock:
                received['mux_outputs'] += 1
                if any(not math.isfinite(value) or value != 0. for value in vector):
                    received['mux_nonzero_outputs'] += 1

        def transform_callback(message):
            caller = message._connection_header.get('callerid', 'unknown')
            with lock:
                for transform in message.transforms:
                    edge = transform.header.frame_id+'->'+transform.child_frame_id
                    if edge not in ('map->odom', 'odom->base_link'):
                        continue
                    received['tf_owners'].setdefault(edge, set()).add(caller)
                    p, q = transform.transform.translation, transform.transform.rotation
                    row = dict(x=p.x, y=p.y,
                               yaw=math.atan2(2*q.w*q.z, 1.-2*q.z*q.z), caller=caller)
                    received['map_tf' if edge == 'map->odom' else 'local_tf'] = row

        subscribers.extend([
            rospy.Subscriber('/geographic/status', String, status_callback, queue_size=50),
            rospy.Subscriber('/global/odom', Odometry, global_callback, queue_size=100),
            rospy.Subscriber('/ugv/move_base/cancel', GoalID, cancel_callback, queue_size=10),
            rospy.Subscriber('/gnss/odometry', Odometry, observation_callback, queue_size=100),
            rospy.Subscriber('/cup_nav/manual_state', String, manual_state_callback, queue_size=50),
            rospy.Subscriber('/TEST_ONLY_chain/final', Twist, mux_output_callback, queue_size=100),
            rospy.Subscriber('/tf', TFMessage, transform_callback, queue_size=200)])
        launch(['rosrun', 'ugv_slicer_control', 'velocity_mux.py',
                '_config_file:='+str(workspace/'src/ugv_bringup/config/test_control_chain.yaml'),
                '_manual_release_owner:=/ugv/control'])
        wait_for(lambda: received['manual_state'] is not None and received['mux_outputs'] > 0,
                 'real held Mux state and zero-only test output')
        assert received['manual_state_caller'] == '/slicer_velocity_mux'
        assert received['manual_state']['manual_hold'] is True
        assert received['manual_state']['hold_reason'] == 'startup'
        mux_generation = received['manual_state']['takeover_generation']
        odom_pub = rospy.Publisher('/odom', Odometry, queue_size=20)
        fix_pub = rospy.Publisher('/gnss/fix', NavSatFix, queue_size=20)
        quality_pub = rospy.Publisher('/gnss/fix_quality', DiagnosticArray, queue_size=20)
        broadcaster = tf2_ros.TransformBroadcaster()

        def publish():
            sequence = 0
            while not stop.is_set() and not rospy.is_shutdown():
                now = rospy.Time.now()
                with lock:
                    if state['moving_since'] is not None:
                        state['distance'] = min(2., .5*(time.monotonic()-state['moving_since']))
                    current = dict(state)
                distance = current['distance']
                message = Odometry()
                message.header.stamp, message.header.frame_id, message.child_frame_id = now, 'odom', 'base_link'
                message.pose.pose.position.x = 13. if current['reset'] else 3.
                message.pose.pose.position.y = 4.+distance
                message.pose.pose.orientation.z = math.sin(math.pi/4.)
                message.pose.pose.orientation.w = math.cos(math.pi/4.)
                message.twist.twist.linear.x = current['velocity']
                for index in (0, 7, 35):
                    message.twist.covariance[index] = .01
                    message.pose.covariance[index] = .01
                odom_pub.publish(message)
                transform = TransformStamped()
                transform.header = message.header
                transform.child_frame_id = 'base_link'
                transform.transform.translation.x = message.pose.pose.position.x
                transform.transform.translation.y = message.pose.pose.position.y
                transform.transform.rotation = message.pose.pose.orientation
                broadcaster.sendTransform(transform)
                if current['fixes'] and sequence % 2 == 0:
                    # Origin (0,0,0), ECEF=(a,east,0) gives exact geodetic
                    # latitude=0, longitude=atan2(east,a), ellipsoid height=r-a.
                    east = distance+.07+current['shift']
                    fix = NavSatFix()
                    fix.header.stamp = now-rospy.Duration(5.) if current['stale'] else now
                    fix.header.frame_id = 'gnss_link'
                    fix.status.status = 2  # q4 and q5 are both standard GBAS_FIX.
                    fix.latitude = 0.
                    fix.longitude = math.degrees(math.atan2(east, 6378137.))
                    fix.altitude = math.hypot(6378137., east)-6378137.
                    fix.position_covariance_type = 0  # exercise explicit conservative assumption
                    quality = DiagnosticArray()
                    quality.header.stamp = fix.header.stamp
                    status = DiagnosticStatus(name='cup_gnss/receiver', hardware_id='TEST_ONLY')
                    metadata = dict(gga_quality=str(current['quality']),
                                    fresh_rtk_usable='true' if current['quality'] == 4 else 'false',
                                    fix_valid='true', session='TEST_ONLY-current-session',
                                    gga_utc_time=str(sequence), invalid_reason='')
                    status.values = [KeyValue(key=key, value=value) for key, value in metadata.items()]
                    quality.status = [status]
                    fix_pub.publish(fix)
                    quality_pub.publish(quality)
                sequence += 1
                stop.wait(.05)

        thread = threading.Thread(target=publish, daemon=True)
        thread.start()
        fixture = Path(temporary.name)
        site = dict(datum='WGS84', origin=dict(latitude=0., longitude=0., altitude_ellipsoid_m=0.),
                    antenna_in_base_link_m=[.07, 0., 0.])
        site_path, session_path = fixture/'site.yaml', fixture/'session.json'
        site_path.write_text(yaml.safe_dump(site))
        launch(['roslaunch', str(workspace/'src/ugv_bringup/launch/slicer_geographic.launch'),
                'site_config:='+str(site_path), 'session_file:='+str(session_path)])
        wait_for(lambda: received['status'] is not None, 'production geographic status', 12.)
        rospy.wait_for_service('/geographic/begin_registration', timeout=5.)
        rospy.wait_for_service('/geographic/finish_registration', timeout=5.)
        rospy.wait_for_service('/global_localization/set_pose', timeout=5.)
        result['actual_set_pose_service'] = '/global_localization/set_pose'
        begin = rospy.ServiceProxy('/geographic/begin_registration', Trigger)()
        assert begin.success, begin.message
        phase(2.)
        with lock:
            state.update(velocity=.5, moving_since=time.monotonic())
        phase(4.05)
        with lock:
            state.update(velocity=0., moving_since=None, distance=2.)
        phase(2.)
        finished = rospy.ServiceProxy('/geographic/finish_registration', Trigger)()
        result['registration'] = dict(success=finished.success, message=finished.message)
        assert finished.success, finished.message
        alignment = json.loads(finished.message)
        assert abs(alignment['yaw']+math.pi/2.) < .03, alignment
        assert math.hypot(alignment['translation'][0]+4., alignment['translation'][1]-3.) < .1, alignment
        wait_for(lambda: received['globals'] and received['map_tf'] and
                 received['status']['state'] == 'ready', 'registered real global odom and TF')
        phase(2.)
        with lock:
            baseline = dict(global_pose=dict(received['globals'][-1]), map_tf=dict(received['map_tf']),
                            local_tf=dict(received['local_tf']), origin=received['status']['origin'])
            state['shift'] = .4
        phase(3.)
        with lock:
            corrected = dict(global_pose=dict(received['globals'][-1]), map_tf=dict(received['map_tf']),
                             local_tf=dict(received['local_tf']))
        global_change = corrected['global_pose']['x']-baseline['global_pose']['x']
        tf_change = math.hypot(corrected['map_tf']['x']-baseline['map_tf']['x'],
                               corrected['map_tf']['y']-baseline['map_tf']['y'])
        local_change = math.hypot(corrected['local_tf']['x']-baseline['local_tf']['x'],
                                  corrected['local_tf']['y']-baseline['local_tf']['y'])
        result['persistent_correction'] = dict(baseline=baseline, corrected=corrected,
                                              global_x_change_m=global_change,
                                              map_odom_translation_change_m=tf_change,
                                              local_translation_change_m=local_change)
        assert global_change > .08 and tf_change > .08, result['persistent_correction']
        assert local_change < 1e-9, result['persistent_correction']
        assert abs(corrected['local_tf']['yaw']-baseline['local_tf']['yaw']) < 1e-9

        with lock:
            state['quality'] = 5
        phase(.3)  # drain any q4 already in flight
        before = received['observations']
        rejected_before = received['status']['rejected_count']
        phase(.8)
        assert received['observations'] == before, 'q5 incorrectly reached global fusion'
        assert received['status']['rejected_count'] > rejected_before
        result['q5_rejection'] = dict(observations_during_q5=received['observations']-before,
                                      reason=received['status']['reason'])
        with lock:
            state['fixes'] = False
        # The production receiver also discards reordered epochs. Advance past
        # that watermark so the next now-5s sample tests age independently.
        phase(5.2)
        with lock:
            state.update(quality=4, stale=True, fixes=True)
        phase(.3)
        before = received['observations']
        rejected_before = received['status']['rejected_count']
        phase(.8)
        assert received['observations'] == before, 'stale fix incorrectly reached global fusion'
        assert received['status']['rejected_count'] > rejected_before
        result['stale_rejection'] = dict(observations_during_stale=received['observations']-before,
                                         reason=received['status']['reason'],
                                         silence_before_stale_s=5.2)
        with lock:
            state.update(fixes=False, stale=False)
        phase(3.)
        loss = dict(received['status'])
        assert loss['registered'] and loss['state'] == 'degraded', loss
        assert loss['origin'] == baseline['origin']
        result['short_loss'] = dict(state=loss['state'], reason=loss['reason'], origin=loss['origin'])
        before = received['observations']
        with lock:
            state['fixes'] = True
        wait_for(lambda: received['observations'] > before and received['status']['state'] == 'ready',
                 'qualified RTK recovery')
        assert received['status']['origin'] == baseline['origin']
        result['recovery'] = dict(state=received['status']['state'], origin=received['status']['origin'],
                                  observations_after_recovery=received['observations']-before)
        generation = received['status']['generation']
        with lock:
            state.update(fixes=False, reset=True)
        wait_for(lambda: received['status']['generation'] > generation and received['cancel'] > 0,
                 'odom reset invalidation and cancel')
        reset = dict(received['status'])
        assert not reset['registered'] and reset['state'] == 'invalid', reset
        assert reset['reason'] == 'odom_position_reset', reset
        phase(5.5)  # let the production hold attempt finish or its 5-second timeout fire
        acknowledgement = dict(received['manual_state'])
        assert received['manual_state_caller'] == '/slicer_velocity_mux'
        assert acknowledgement['last_intent_action'] == 'takeover', acknowledgement
        assert acknowledgement['last_intent_accepted'] is True, acknowledgement
        assert acknowledgement['hold_reason'] == 'manual_takeover', acknowledgement
        assert acknowledgement['manual_hold'] is True, acknowledgement
        assert acknowledgement['takeover_generation'] == mux_generation+1, acknowledgement
        assert received['mux_nonzero_outputs'] == 0
        assert received['mux_unheld_states'] == 0
        result['hold_ack'] = dict(caller=received['manual_state_caller'],
                                initial_takeover_generation=mux_generation,
                                manual_state=acknowledgement,
                                output_topic='/TEST_ONLY_chain/final',
                                zero_twist_messages=received['mux_outputs'],
                                nonzero_twist_messages=received['mux_nonzero_outputs'],
                                observed_unheld_states=received['mux_unheld_states'])
        result['odom_reset'] = dict(state=reset['state'], reason=reset['reason'],
                                    cancel_messages=received['cancel'], generation=reset['generation'])
        owners = {edge: sorted(nodes) for edge, nodes in received['tf_owners'].items()}
        assert set(owners) == {'map->odom', 'odom->base_link'}, owners
        assert all(len(nodes) == 1 for nodes in owners.values()), owners
        assert owners['map->odom'] == ['/global_localization/ekf'], owners
        result['dynamic_tf_publishers'] = owners
        result['saved_session'] = json.loads(session_path.read_text())
        result['passed'] = True
    except Exception as error:
        result.update(error=str(error), traceback=traceback.format_exc())
        traceback.print_exc(file=log)
    finally:
        stop.set()
        if thread:
            thread.join(timeout=2.)
        try:
            rospy.signal_shutdown('TEST_ONLY geographic probe finished')
        except NameError:
            pass
        for process in reversed(processes):
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=5.)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=2.)
        temporary.cleanup()
        result['elapsed_s'] = round(time.monotonic()-started, 3)
        result['log'] = str(log_path)
        log.close()
        runtime_log = log_path.read_text()
        result['hold_attempt_log'] = [line for line in runtime_log.splitlines() if 'Existing hold ' in line]
        result['requested_time_tf_fallback_warnings'] = runtime_log.count(
            'Transform from odom to base_link was unavailable for the time requested. Using latest instead.')
        report_path.write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
        print(json.dumps(result, indent=2, allow_nan=False))
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
