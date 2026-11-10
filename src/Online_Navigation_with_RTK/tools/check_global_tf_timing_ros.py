#!/usr/bin/env python3
"""TEST_ONLY actual RL map/odom timing; zero-velocity synthetic inputs only.

Run with tools/noetic.sh (--network none, no devices). The config is loaded
unchanged into one private-master EKF. No driver, Twist, Action or Mux is used.
The success assertion intentionally fails for the lagging baseline config.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import threading
import time
import xmlrpc.client


def main():
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=root/'src/ugv_bringup/config/global_ekf.yaml')
    parser.add_argument('--output', type=Path, default=root/'.test-results/global-tf-timing.json')
    args = parser.parse_args()
    if (not Path('/.dockerenv').exists() or
            {p.name for p in Path('/sys/class/net').iterdir()} != {'lo'}):
        raise RuntimeError('TEST_ONLY requires Docker --network none via tools/noetic.sh')
    if any(Path('/dev').glob('ttyACM*')) or any(Path('/dev').glob('ttyUSB*')) or Path('/dev/c30d').exists():
        raise RuntimeError('TEST_ONLY requires no mapped hardware devices')
    import yaml
    config = yaml.safe_load(args.config.read_text())
    assert config['world_frame'] == 'map' and config['odom_frame'] == 'odom'
    assert config['base_link_frame'] == 'base_link' and config['publish_tf'] is True
    args.output.parent.mkdir(parents=True, exist_ok=True)
    log = args.output.with_suffix('.log').open('w')
    os.environ.update(ROS_MASTER_URI='http://127.0.0.1:12429', ROS_IP='127.0.0.1')
    os.environ.pop('ROS_HOSTNAME', None)
    children, subscribers = [], []
    stop, lock = threading.Event(), threading.Lock()
    thread, ros = None, None
    report = dict(TEST_ONLY=True, network='none', no_devices=True, passed=False,
                  config=str(args.config), config_sha256=hashlib.sha256(args.config.read_bytes()).hexdigest(),
                  predict_to_current_time=config.get('predict_to_current_time', False),
                  transform_time_offset=config.get('transform_time_offset', 0.),
                  synthetic_local_tf_age_s=.015, synthetic_motion_age_s=.110,
                  warmup_s=3., sample_s=5., physical_validation=False)

    def launch(command):
        proc = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        children.append(proc)
        return proc

    def wait_for(predicate, message, seconds=8.):
        deadline = time.monotonic()+seconds
        while time.monotonic() < deadline:
            if predicate():
                return
            if any(p.poll() is not None for p in children):
                raise AssertionError('Child exited: '+message)
            time.sleep(.025)
        raise AssertionError('Timed out: '+message)

    def stats(rows):
        ages = sorted(row['age_s'] for row in rows)
        return dict(count=len(rows), hz=(len(rows)-1)/(rows[-1]['received']-rows[0]['received']),
                    min_age_s=ages[0], p95_age_s=ages[min(len(ages)-1, int(len(ages)*.95))],
                    max_age_s=ages[-1], first_stamp=rows[0]['stamp'], last_stamp=rows[-1]['stamp'])

    try:
        launch(['roscore', '-p', '12429'])
        master = xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI'])

        def master_ready():
            try:
                return master.getPid('/TEST_ONLY_global_tf_probe')[0] == 1
            except (OSError, xmlrpc.client.Error):
                return False

        wait_for(master_ready, 'private master')
        import rospy
        import tf2_ros
        from geometry_msgs.msg import TransformStamped
        from nav_msgs.msg import Odometry
        from tf2_msgs.msg import TFMessage
        ros = rospy
        rospy.init_node('TEST_ONLY_global_tf_probe', disable_signals=True)
        rospy.set_param('/use_sim_time', False)
        rospy.set_param('/TEST_ONLY_global_tf_ekf', config)
        globals_, transforms = [], []

        def global_odom(message):
            point, q = message.pose.pose.position, message.pose.pose.orientation
            row = dict(received=time.monotonic(), stamp=message.header.stamp.to_sec(),
                       age_s=rospy.Time.now().to_sec()-message.header.stamp.to_sec(),
                       x=point.x, y=point.y, yaw=math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))
            with lock:
                globals_.append(row)

        def tf_message(message):
            for transform in message.transforms:
                if transform.header.frame_id == 'map' and transform.child_frame_id == 'odom':
                    row = dict(received=time.monotonic(), stamp=transform.header.stamp.to_sec(),
                               age_s=rospy.Time.now().to_sec()-transform.header.stamp.to_sec())
                    with lock:
                        transforms.append(row)

        subscribers.extend([rospy.Subscriber('/TEST_ONLY/global/odom', Odometry, global_odom, queue_size=100),
                            rospy.Subscriber('/tf', TFMessage, tf_message, queue_size=100)])
        buffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(buffer)
        broadcaster = tf2_ros.TransformBroadcaster()
        motion_pub = rospy.Publisher(config['odom0'], Odometry, queue_size=20)
        launch(['rosrun', 'robot_localization', 'ekf_localization_node',
                '__name:=TEST_ONLY_global_tf_ekf', 'odometry/filtered:=/TEST_ONLY/global/odom'])

        def publish():
            while not stop.is_set() and not rospy.is_shutdown():
                now = rospy.Time.now()
                transform = TransformStamped()
                transform.header.frame_id, transform.child_frame_id = 'odom', 'base_link'
                transform.header.stamp = now-rospy.Duration(.015)
                transform.transform.rotation.w = 1.
                broadcaster.sendTransform(transform)
                message = Odometry()
                message.header.frame_id, message.child_frame_id = 'odom', 'base_link'
                message.header.stamp = now-rospy.Duration(.110)
                message.pose.pose.orientation.w = 1.
                for i in (0, 7, 35):
                    message.twist.covariance[i] = .01
                motion_pub.publish(message)
                stop.wait(.05)

        thread = threading.Thread(target=publish, daemon=True)
        thread.start()
        wait_for(lambda: len(globals_) >= 3 and len(transforms) >= 3, 'actual global EKF output')
        time.sleep(3.)
        start = time.monotonic()
        queries = []
        while time.monotonic()-start < 5.:
            stamp = buffer.lookup_transform('odom', 'base_link', rospy.Time(0)).header.stamp
            row = dict(local_stamp=stamp.to_sec(), received=time.monotonic())
            try:
                transform = buffer.lookup_transform('map', 'odom', stamp)
                row.update(success=True, result_stamp=transform.header.stamp.to_sec())
            except tf2_ros.TransformException as error:
                row.update(success=False, error=str(error))
            queries.append(row)
            time.sleep(.05)
        with lock:
            global_rows = [r for r in globals_ if r['received'] >= start]
            tf_rows = [r for r in transforms if r['received'] >= start]
        assert len(global_rows) >= 50 and len(tf_rows) >= 50, 'Insufficient actual EKF samples'
        report.update(global_odom=stats(global_rows), map_odom_tf=stats(tf_rows),
                      lookup=dict(count=len(queries), success=sum(r['success'] for r in queries),
                                  failed=sum(not r['success'] for r in queries)), queries=queries,
                      pose=dict(max_xy_step_m=max(math.hypot(b['x']-a['x'], b['y']-a['y'])
                                                 for a, b in zip(global_rows, global_rows[1:])),
                                max_abs_xy_m=max(math.hypot(r['x'], r['y']) for r in global_rows),
                                max_abs_yaw_rad=max(abs(r['yaw']) for r in global_rows)))
        report['robot_localization_version'] = subprocess.check_output(
            ['rosversion', 'robot_localization'], text=True).strip()
        assert report['lookup']['success'] == report['lookup']['count'], report['lookup']
        assert report['pose']['max_abs_xy_m'] < 1e-9 and report['pose']['max_abs_yaw_rad'] < 1e-9
        report['passed'] = True
    except Exception as error:
        report['error'] = str(error)
    finally:
        stop.set()
        if thread:
            thread.join(timeout=2.)
        for subscriber in subscribers:
            subscriber.unregister()
        if ros:
            ros.signal_shutdown('TEST_ONLY TF timing probe complete')
        for child in reversed(children):
            if child.poll() is None:
                os.killpg(child.pid, signal.SIGINT)
                try:
                    child.wait(timeout=3.)
                except subprocess.TimeoutExpired:
                    os.killpg(child.pid, signal.SIGKILL)
                    child.wait()
        log.close()
        args.output.write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps({k: v for k, v in report.items() if k != 'queries'}, indent=2))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
