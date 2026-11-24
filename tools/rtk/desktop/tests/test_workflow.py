"""Mock the desktop process runner and the global ROS graph."""
import json
from pathlib import Path
import sys
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[4] / 'src/ugv_navigation/scripts'))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from dashboard_model import Dashboard
from workflow import OwnedProcess, Workflow
from workflow_discovery import Graph


class Process:
    def __init__(self):
        self.status = None
        self.stopped = False
        self.log_lines = []
        self.sent = []

    def poll(self):
        return self.status

    def stop(self):
        self.stopped = True

    def lines(self):
        result, self.log_lines = self.log_lines, []
        return result

    def send(self, command):
        self.sent.append(command)


class OwnedProcessTests(unittest.TestCase):
    def test_stop_tracks_group_after_serial_wrapper_exits(self):
        child = OwnedProcess.__new__(OwnedProcess)
        child.process = Mock(pid=2345)
        child.process.poll.return_value = -15
        child.stop_at = 1.
        child.signal_stage = 1
        with patch('owned_process.time.monotonic', return_value=28.), patch('owned_process.os.killpg') as kill:
            self.assertIsNone(child.poll())
            kill.assert_called_once_with(2345, 0)
            kill.side_effect = ProcessLookupError
            self.assertEqual(child.poll(), -15)


class WorkflowTests(unittest.TestCase):
    def setUp(self):
        self.board = Dashboard()
        self.graph = Graph(online=True)
        self.starts = []
        self.clock = 10.
        self.env = dict(HOME='/test', DISPLAY=':17', XAUTHORITY='/x',
                        ROS_MASTER_URI='http://localhost:11999', ROS_IP='127.0.0.1',
                        ROS_NAMESPACE='/desktop')
        self.workflow = Workflow(self.board, self.spawn, self.env, lambda: self.clock)
        self.refresh()

    def spawn(self, key, argv, env):
        proc = Process()
        self.starts.append((key, argv, env, proc))
        return proc

    def refresh(self):
        self.workflow.update(self.graph)

    def sample(self, **samples):
        for key, value in samples.items():
            self.board.record(key, value, self.clock)

    def prerequisites(self, fixed=True, held=True):
        self.graph.nodes.update(('/geographic', '/slicer_c30d_driver', '/ugv/move_base'))
        self.graph.frame = 'odom'
        self.graph.subscribers['/ugv/move_base/goal'] = {'/ugv/move_base'}
        self.graph.publishers['/cup_nav/manual_state'] = {'/slicer_velocity_mux'}
        self.sample(lidar=True, odom=(0., 0.), driver=True, action=[],
                    odom_still=True, geographic={'state': 'invalid'},
                    mux={'manual_hold': held, 'teleop_fresh': False, 'emergency_stop_latched': False},
                    quality={'gga_quality': 4 if fixed else 5, 'fresh_rtk_usable': str(fixed).lower()},
                    fixed_position={'latitude': 19., 'longitude': 119., 'altitude': 7.} if fixed else None)
        self.refresh()

    def color(self, key):
        return self.workflow.states[key][0]

    def test_sensor_driver_and_odom_lifecycle(self):
        self.assertEqual(self.color('sensors'), 'red')
        self.workflow.click('sensors')
        self.assertEqual(self.starts[-1][1][-1], 'config_file:=/test/.config/slicer/gnss.yaml')
        self.assertEqual(self.color('sensors'), 'yellow')
        self.graph.nodes.add('/geographic')
        self.sample(lidar=True, odom=(0, 0))
        self.refresh()
        self.assertEqual(self.color('sensors'), 'green')
        self.workflow.click('driver')
        self.assertEqual(self.starts[-1][1][-1], 'slicer_body_driver.launch')
        self.sample(driver=True)
        self.refresh()
        self.assertEqual(self.color('driver'), 'green')
        self.workflow.click('navigation')
        self.assertEqual(self.starts[-1][1], ('roslaunch', 'ugv_bringup', 'slicer_navigation_runtime.launch'))
        self.graph.frame = 'odom'
        self.graph.subscribers['/ugv/move_base/goal'] = {'/ugv/move_base'}
        self.sample(action=[])
        self.refresh()
        self.assertEqual(self.workflow.states['navigation'][:2], ('yellow', '等待 RTK Fixed'))
        self.workflow.click('navigation')
        self.starts[-1][3].status = 0
        self.graph.frame = None
        self.graph.subscribers.clear()
        self.refresh()
        self.assertEqual(self.color('navigation'), 'red')

    def test_rtk_start_waits_for_fresh_stationary_inputs_then_runs_roundtrip_once(self):
        self.workflow.click('navigation')
        self.prerequisites(fixed=False, held=False)
        self.assertTrue(self.workflow.startup_calibration_pending)
        self.sample(lio=True, wheel_still=False)
        self.refresh()
        self.assertEqual([entry[0] for entry in self.starts], ['navigation'])
        self.prerequisites(held=True)
        self.sample(wheel_still=True)
        self.refresh()
        self.assertEqual([entry[0] for entry in self.starts], ['navigation'])

        self.sample(mux=dict(self.workflow.mux, manual_hold=False))
        self.refresh()
        command = self.starts[-1][1]
        self.assertEqual(command, ('rosrun', 'ugv_navigation', 'geographic.py',
                                   'initialize', '--distance', '2.0', '--return-to-start'))
        for _ in range(3):
            self.refresh()
        self.assertEqual([entry[0] for entry in self.starts], ['navigation', 'calibrate'])

        self.sample(geographic={'state': 'ready', 'registered': True})
        self.starts[-1][3].status = 0
        self.refresh()
        self.assertEqual(self.starts[-1][1], ('rosservice', 'call', '/geographic/navigation_map'))
        self.starts[-1][3].status = 0
        self.graph.frame = 'map'
        self.refresh()
        self.assertFalse(self.workflow.startup_calibration_pending)

    def test_roundtrip_failure_does_not_restart_or_switch_even_when_outbound_registered(self):
        self.workflow.click('navigation')
        self.prerequisites(held=False)
        self.sample(lio=True, wheel_still=True)
        self.refresh()
        self.assertEqual(self.starts[-1][0], 'calibrate')
        self.sample(geographic={'state': 'ready', 'registered': True})
        self.starts[-1][3].status = 1
        for _ in range(3):
            self.refresh()
        self.assertEqual([entry[0] for entry in self.starts], ['navigation', 'calibrate'])
        self.assertIn('未完成', self.workflow.states['navigation'][1])
        self.assertFalse(self.workflow.startup_calibration_pending)

    def test_observing_external_rtk_does_not_automatically_move(self):
        self.prerequisites(held=False)
        self.sample(lio=True, wheel_still=True)
        self.refresh()
        self.assertFalse(self.workflow.startup_calibration_pending)
        self.assertEqual(self.starts, [])

    def test_stop_all_clears_pending_startup_and_does_not_launch_when_fix_arrives(self):
        self.workflow.click('navigation')
        self.prerequisites(fixed=False, held=False)
        self.workflow.stop_all()
        self.prerequisites(held=False)
        self.sample(lio=True, wheel_still=True)
        self.refresh()
        self.assertFalse(self.workflow.startup_calibration_pending)
        self.assertEqual([entry[0] for entry in self.starts], ['navigation'])

    def test_external_ready_and_stale_owner_both_block_duplicate(self):
        self.prerequisites()
        expected = {'sensors': '已就绪', 'driver': '已完成', 'navigation': '外部运行'}
        for key in ('sensors', 'driver', 'navigation'):
            self.assertEqual(self.workflow.states[key][:2], ('green', expected[key]))
            self.workflow.click(key)
        self.assertEqual(self.starts, [])
        self.clock += 10.
        self.refresh()
        self.assertEqual(self.color('navigation'), 'yellow')
        self.workflow.click('navigation')
        self.assertEqual(self.starts, [])

    def test_partial_external_sensor_stack_reports_missing_data_and_allows_retry_after_exit(self):
        self.graph.nodes.add('/slicer_ekf')
        self.refresh()
        self.assertEqual(self.workflow.states['sensors'][:2], ('yellow', '等待雷达扫描'))
        self.workflow.click('sensors')
        self.assertEqual(self.starts, [])
        self.assertIn('等待雷达扫描', self.workflow.messages[-1])
        self.graph.nodes.clear()
        self.refresh()
        self.assertEqual(self.starts, [])
        self.workflow.click('sensors')
        self.assertEqual([key for key, *_ in self.starts], ['sensors'])

    def test_odom_reports_missing_local_odometry_and_unexpected_exit_can_retry(self):
        self.workflow.click('navigation')
        self.assertEqual(self.workflow.states['navigation'][:2], ('yellow', '等待本地里程计'))
        self.starts[-1][3].status = 0
        self.refresh()
        self.assertEqual(self.workflow.states['navigation'][:2], ('red', '进程已退出'))
        self.workflow.click('navigation')
        self.assertEqual([key for key, *_ in self.starts], ['navigation', 'navigation'])

    def test_manual_calibration_requires_released_mux_then_restores_own_hold(self):
        self.prerequisites(fixed=False)
        self.assertIsNone(self.workflow.click('calibrate'))
        self.assertEqual(self.starts, [])
        self.prerequisites()
        self.assertIsNone(self.workflow.click('calibrate'))
        self.assertEqual(self.starts, [])
        self.sample(mux={'manual_hold': False, 'teleop_fresh': False, 'emergency_stop_latched': False,
                         'mux_instance_id': 'mux-a', 'takeover_generation': 0})
        self.assertEqual(self.workflow.click('calibrate'), 'confirm')
        self.workflow.click('calibrate', confirmed=True)
        self.assertEqual(self.color('calibrate'), 'yellow')
        self.assertIn('initialize', self.starts[-1][1])
        self.starts[-1][3].status = 0
        self.sample(geographic={'state': 'ready', 'registered': True})
        self.refresh()
        self.assertIn('hold', self.starts[-1][1])
        self.starts[-1][3].status = 0
        self.refresh()
        self.assertEqual(self.workflow.states['calibrate'][:2], ('green', '已配准'))
        self.sample(geographic={'state': 'invalid'})
        self.refresh()
        self.assertEqual(self.color('calibrate'), 'red')

    def test_diagnostic_map_requests_owned_runtime_switch(self):
        self.workflow.click('sensors')
        self.workflow.click('driver')
        self.workflow.click('navigation')
        self.prerequisites(held=False)
        self.sample(geographic={'state': 'ready', 'registered': True})
        self.workflow.click('map')
        self.assertEqual(self.starts[-1][1], ('rosservice', 'call', '/geographic/navigation_map'))
        self.assertTrue(all(not p.stopped for _, _, _, p in self.starts[:3]))

    def test_diagnostic_map_releases_only_its_own_calibration_hold(self):
        self.workflow.click('navigation')
        self.prerequisites(held=False)
        self.sample(mux=dict(self.workflow.mux, mux_instance_id='mux-a', takeover_generation=0))
        self.workflow.click('calibrate', confirmed=True)
        self.starts[-1][3].status = 0
        self.sample(geographic={'state': 'ready', 'registered': True})
        self.refresh()
        hold = self.starts[-1][3]
        ack = dict(self.workflow.mux, manual_hold=True, takeover_generation=1,
                   last_intent_action='takeover', last_intent_accepted=True)
        hold.log_lines = [json.dumps(ack)]
        hold.status = 0
        self.sample(mux=dict(ack, takeover_generation=2))
        self.refresh()
        self.workflow.click('map')
        self.assertIs(self.starts[-1][3], hold)
        self.sample(mux=ack)
        self.workflow.click('map')
        step, command, _, release = self.starts[-1]
        self.assertEqual(step, 'map_release')
        self.assertEqual(command[-4:], ('--expected-mux-instance', 'mux-a', '--expected-generation', '1'))
        release.status = 0
        self.refresh()
        self.assertEqual(self.starts[-1][1], ('rosservice', 'call', '/geographic/navigation_map'))

    def test_map_refuses_to_stop_external_odom(self):
        self.prerequisites()
        self.sample(geographic={'state': 'ready', 'registered': True})
        self.workflow.click('map')
        self.assertEqual(self.starts, [])
        self.assertIn('外部终端', self.workflow.messages[-1])

    def test_diagnostic_map_starts_managed_runtime_after_external_navigation_stops(self):
        self.prerequisites()
        self.graph.frame = None
        self.graph.nodes.discard('/ugv/move_base')
        self.graph.subscribers.clear()
        self.sample(mux={}, geographic={'state': 'ready', 'registered': True})
        self.workflow.click('map')
        self.assertEqual(self.starts[-1][0:2], ('navigation', (
            'roslaunch', 'ugv_bringup', 'slicer_navigation_runtime.launch', 'initial_mode:=map')))

    def test_stop_all_interrupts_switch_and_stops_only_owned_runtime(self):
        self.workflow.click('navigation')
        self.prerequisites(held=False)
        self.sample(geographic={'state': 'ready', 'registered': True})
        self.workflow.click('map')
        owned_navigation, switch = self.starts[-2][3], self.starts[-1][3]
        self.workflow.stop_all()
        self.assertTrue(switch.stopped)
        self.assertFalse(owned_navigation.stopped)
        switch.status = -2
        self.refresh()
        self.assertTrue(owned_navigation.stopped)

    def test_calibration_can_retry_when_manager_remains_collecting_but_worker_exited(self):
        self.prerequisites(held=False)
        self.sample(geographic={'state': 'collecting'})
        self.graph.nodes.add('/slicer_geographic_client_123')
        self.refresh()
        self.assertEqual(self.color('calibrate'), 'yellow')
        self.assertIsNone(self.workflow.click('calibrate'))
        self.graph.nodes.discard('/slicer_geographic_client_123')
        self.refresh()
        self.assertEqual(self.color('calibrate'), 'red')
        self.assertEqual(self.workflow.click('calibrate'), 'confirm')

    def test_wgs84_requires_fresh_paired_fixed_position_at_submission(self):
        self.prerequisites(held=False)
        self.sample(runtime={'phase': 'IDLE', 'busy': False})
        self.assertTrue(self.workflow.states['wgs84'][2])

        # A diagnostic can still say Fixed after the position pair has expired.
        old_position = {'latitude': 19., 'longitude': 119., 'altitude': 7.}
        self.board.record('fixed_position', old_position, self.clock - 3.)
        self.assertFalse(self.workflow.states['wgs84'][2])
        self.assertFalse(self.workflow.send_wgs84(20., 120.))
        self.assertEqual(self.starts, [])

        self.sample(fixed_position=dict(old_position, altitude=8.25))
        self.assertTrue(self.workflow.send_wgs84('20.2', '120.8'))
        self.assertEqual(self.starts[-1][1][-2:], ('--altitude-ellipsoid', '8.25'))

    def test_wgs84_requires_ready_driver_local_odom_and_unowned_idle_control(self):
        unsafe_states = (
            {'driver': None, 'wheel_odom': True},
            {'driver': False}, {'odom': None}, {'action': [('other', 1)]},
            {'mux': {'manual_hold': True, 'teleop_fresh': False, 'emergency_stop_latched': False}},
            {'mux': {'manual_hold': False, 'teleop_fresh': True, 'emergency_stop_latched': False}},
            {'mux': {'manual_hold': False, 'teleop_fresh': False, 'emergency_stop_latched': True}},
            {'mux': {'manual_hold': False, 'teleop_fresh': False, 'emergency_stop_latched': False,
                     'selected_source': 'mission'}},
            {'mux': None},
        )
        for unsafe in unsafe_states:
            with self.subTest(unsafe=unsafe):
                self.prerequisites(held=False)
                self.sample(runtime={'phase': 'IDLE', 'busy': False}, **unsafe)
                self.assertFalse(self.workflow.states['wgs84'][2])
                self.assertFalse(self.workflow.send_wgs84(20., 120.))
        self.assertEqual(self.starts, [])

    def test_wgs84_rejects_malformed_or_out_of_range_coordinates(self):
        self.prerequisites(held=False)
        self.sample(runtime={'phase': 'IDLE', 'busy': False})
        for lat, lon in (('', '120'), ('nan', '120'), ('20', 'inf'), ('91', '120'), ('20', '-181')):
            with self.subTest(lat=lat, lon=lon):
                self.assertFalse(self.workflow.send_wgs84(lat, lon))
        self.assertEqual(self.starts, [])

    def test_owned_wgs84_can_be_canceled_after_loss_of_fixed_position(self):
        self.prerequisites(held=False)
        self.sample(runtime={'phase': 'IDLE', 'busy': False})
        self.assertTrue(self.workflow.send_wgs84(20., 120.))
        self.sample(fixed_position=None)
        self.assertEqual(self.workflow.states['wgs84'], ('yellow', '取消本地任务', True))
        self.workflow.stop('wgs84')
        self.assertTrue(self.starts[-1][3].stopped)

    def test_stop_all_cancels_local_wgs84_before_runtime(self):
        self.workflow.click('navigation')
        navigation = self.starts[-1][3]
        self.prerequisites(held=False)
        self.sample(runtime={'phase': 'IDLE', 'busy': False})
        self.assertTrue(self.workflow.send_wgs84(20., 120.))
        key, command, _, sender = self.starts[-1]
        self.assertEqual(key, 'wgs84')
        self.assertEqual(command, ('rosrun', 'ugv_navigation', 'geographic.py', 'send',
                                   '--latitude', '20.0', '--longitude', '120.0',
                                   '--altitude-ellipsoid', '7.0'))
        self.workflow.stop_all()
        self.assertTrue(sender.stopped)
        self.assertFalse(navigation.stopped)
        sender.status = -2
        self.refresh()
        self.assertTrue(navigation.stopped)

    def test_failed_local_task_retains_its_destination_without_running_it_again(self):
        self.workflow.click('navigation')
        self.prerequisites(held=False)
        self.sample(runtime={'phase': 'IDLE', 'busy': False})
        self.assertTrue(self.workflow.send_wgs84(20., 120.))
        self.starts[-1][3].status = 1
        self.refresh()
        self.assertIsNone(self.board.local_target())
        self.assertEqual(self.board.recent('wgs84_failure', self.clock), {'latitude': 20., 'longitude': 120.})
        self.assertEqual([key for key, *_ in self.starts], ['navigation', 'wgs84'])

    def test_local_wgs84_can_restart_preparation_after_map_odom_epoch_expires(self):
        self.workflow.click('navigation')
        self.prerequisites(held=False)
        self.graph.frame = 'map'
        self.sample(runtime={'phase': 'IDLE', 'mode': 'map', 'busy': False},
                    geographic={'state': 'invalid', 'registered': False})
        self.assertFalse(self.workflow.ready('navigation'))
        self.assertTrue(self.workflow.send_wgs84(20., 120.))
        self.assertEqual([key for key, *_ in self.starts], ['navigation', 'wgs84'])

    def test_auto_preparation_blocks_grasp_and_local_wgs84(self):
        self.workflow.click('driver')
        self.workflow.click('navigation')
        self.prerequisites(held=False)
        self.sample(runtime={'phase': 'WAITING_RTK', 'busy': True})
        self.workflow.click('vision')
        self.assertFalse(self.starts[0][3].stopped)
        self.assertFalse(self.workflow.send_wgs84(20., 120.))
        self.assertEqual([key for key, *_ in self.starts], ['driver', 'navigation'])

    def test_new_external_local_process_discovered_again_at_click(self):
        self.workflow.click('vision', process_groups={'vision': {820}})
        self.assertEqual(self.starts, [])
        self.assertIn('外部终端', self.workflow.messages[-1])

    def test_fresh_external_navigation_blocks_individual_driver_stop(self):
        self.workflow.click('driver')
        driver = self.starts[-1][3]
        self.workflow.click('driver', process_groups={'nav': {820}, 'odom': {820}})
        self.assertFalse(driver.stopped)
        self.assertIn('请先停止导航', self.workflow.messages[-1])

    def test_first_grasp_click_runs_headless_and_second_click_releases(self):
        self.workflow.click('vision')
        key, argv, env, proc = self.starts[-1]
        self.assertEqual(key, 'vision')
        self.assertEqual(argv, ('rosrun', 'slicer_visual_grasp', 'run.sh', '--run',
                                '--headless', '--pick-on-start'))
        self.assertNotIn('ROS_NAMESPACE', env)
        self.assertEqual(self.color('vision'), 'yellow')
        self.workflow.click('vision')
        self.assertEqual(proc.sent, [])
        proc.log_lines = ['GRASP_STATE held']
        self.refresh()
        self.assertEqual(self.color('vision'), 'green')
        self.workflow.click('vision')
        self.assertEqual(proc.sent, ['release'])
        proc.log_lines = ['GRASP_STATE idle']
        self.refresh()
        self.assertTrue(proc.stopped)
        proc.status = 0
        self.refresh()
        self.assertEqual(self.workflow.states['vision'][:2], ('red', '已释放'))

    def test_failed_worker_with_possible_attachment_offers_release_first(self):
        self.workflow.click('vision')
        proc = self.starts[-1][3]
        proc.log_lines = ['GRASP_STATE held']
        self.refresh()
        proc.status = 1
        self.refresh()
        self.assertEqual(self.workflow.states['vision'][:2], ('yellow', '需释放'))
        self.workflow.click('vision')
        another = self.starts[-1][3]
        self.assertEqual(self.starts[-1][1], ('rosrun', 'slicer_visual_grasp', 'run.sh',
                                             '--run', '--headless'))
        another.log_lines = ['GRASP_STATE recovery']
        self.refresh()
        self.assertEqual(another.sent, ['release'])

    def test_grasp_does_not_start_under_existing_serial_owner(self):
        self.graph.publishers['/wheel/odom'] = {'/slicer_c30d_driver'}
        self.workflow.click('vision')
        self.assertEqual(self.starts, [])
        self.assertEqual(self.workflow.states['vision'][:2], ('yellow', '底盘由外部终端占用'))
        self.graph.publishers.clear()
        self.workflow.click('vision')
        self.assertEqual(self.starts[-1][0], 'vision')

    def test_grasp_completion_restores_driver_and_allows_nav_while_held(self):
        self.workflow.click('driver')
        old_driver = self.starts[-1][3]
        self.graph.nodes.add('/slicer_c30d_driver')
        self.graph.publishers['/wheel/odom'] = {'/slicer_c30d_driver'}
        self.sample(driver=True, wheel_still=False)
        self.refresh()
        self.workflow.click('vision')
        self.assertFalse(old_driver.stopped)
        self.sample(wheel_still=True)
        self.workflow.click('vision')
        self.assertTrue(old_driver.stopped)
        self.assertEqual([key for key, *_ in self.starts], ['driver'])
        old_driver.status = 0
        self.refresh()
        self.assertEqual([key for key, *_ in self.starts], ['driver'])
        self.graph.nodes.discard('/slicer_c30d_driver')
        self.graph.publishers.clear()
        self.refresh()
        self.assertEqual([key for key, *_ in self.starts], ['driver', 'vision'])
        grasp = self.starts[-1][3]
        self.graph.processes['vision'] = {123}
        grasp.log_lines = ['GRASP_STATE held']
        self.refresh()
        self.assertEqual(self.workflow.states['vision'], ('green', '抓取完成', True))
        self.assertEqual([key for key, *_ in self.starts], ['driver', 'vision', 'driver'])
        self.assertEqual(self.color('driver'), 'yellow')
        self.graph.nodes.add('/geographic')
        self.sample(driver=True, lidar=True, odom=(0., 0.))
        self.workflow.click('navigation')
        self.assertEqual(self.starts[-1][0], 'navigation')
        self.assertFalse(grasp.stopped)
        self.assertEqual(grasp.sent, [])

    def test_release_with_idle_navigation_keeps_navigation_running(self):
        self.workflow.click('driver')
        self.sample(driver=True, wheel_still=True)
        self.workflow.click('vision')
        self.starts[-1][3].status = 0
        self.refresh()
        grasp = self.starts[-1][3]
        grasp.log_lines = ['GRASP_STATE held']
        self.refresh()
        self.assertEqual(self.starts[-1][0], 'driver')
        restarted = self.starts[-1][3]
        self.graph.nodes.add('/geographic')
        self.sample(driver=True, lidar=True, odom=(0., 0.))
        self.workflow.click('navigation')
        self.assertEqual(self.starts[-1][0], 'navigation')
        nav = self.starts[-1][3]
        self.prerequisites(held=True)
        mux = dict(manual_hold=True, teleop_fresh=False, emergency_stop_latched=False,
                   mux_instance_id='existing-mux', takeover_generation=2)
        self.sample(wheel_still=True, mux=mux)
        self.workflow.click('vision')
        self.assertTrue(restarted.stopped)
        self.assertEqual(grasp.sent, [])
        restarted.status = 0
        self.graph.nodes.remove('/slicer_c30d_driver')
        self.refresh()
        self.assertEqual(grasp.sent, ['release'])
        grasp.log_lines = ['GRASP_STATE idle']
        self.refresh()
        self.assertTrue(grasp.stopped)
        grasp.status = 0
        self.refresh()
        self.assertEqual([key for key, *_ in self.starts], ['driver', 'vision', 'driver', 'navigation', 'driver'])
        self.assertFalse(nav.stopped)
        self.assertEqual(self.workflow.states['vision'][:2], ('red', '已释放'))

    def test_grasp_parks_released_mux_and_restores_only_its_own_hold(self):
        self.workflow.click('driver')
        driver = self.starts[-1][3]
        self.prerequisites(held=False)
        mux = dict(manual_hold=False, teleop_fresh=False, emergency_stop_latched=False,
                   mux_instance_id='mux', takeover_generation=2)
        self.sample(mux=mux, wheel_still=True, action=[('moving', 1)])
        self.workflow.click('vision')
        self.assertEqual([key for key, *_ in self.starts], ['driver'])
        self.sample(action=[])
        self.workflow.click('vision')
        key, argv, _, hold = self.starts[-1]
        self.assertEqual(key, 'grasp_hold')
        self.assertIn('--require-idle', argv)
        self.assertFalse(driver.stopped)
        held = dict(mux, manual_hold=True, takeover_generation=3, last_intent_action='takeover',
                    last_intent_accepted=True)
        hold.log_lines = [json.dumps(held)]
        hold.status = 0
        self.refresh()
        self.assertFalse(driver.stopped)
        self.sample(mux=held)
        self.refresh()
        self.assertTrue(driver.stopped)
        driver.status = 0
        self.graph.nodes.remove('/slicer_c30d_driver')
        self.refresh()
        self.assertEqual(self.starts[-1][0], 'vision')
        self.starts[-1][3].log_lines = ['GRASP_STATE held']
        self.refresh()
        self.assertEqual(self.starts[-1][0], 'driver')
        self.sample(driver=True, wheel_still=True)
        self.refresh()
        self.assertEqual(self.starts[-1][0:2], ('grasp_unhold', (
            'rosrun', 'ugv_slicer_control', 'control.py', 'release', '--timeout', '4',
            '--expected-mux-instance', 'mux', '--expected-generation', '3')))

    def test_startup_failure_before_worker_reports_state_restores_driver(self):
        self.workflow.click('driver')
        driver = self.starts[-1][3]
        self.sample(driver=True, wheel_still=True)
        self.workflow.click('vision')
        driver.status = 0
        self.refresh()
        self.starts[-1][3].status = 127
        self.refresh()
        self.assertEqual([key for key, *_ in self.starts], ['driver', 'vision', 'driver'])

    def test_grasp_cannot_stop_driver_while_owned_navigation_is_starting(self):
        self.workflow.click('driver')
        driver = self.starts[-1][3]
        self.graph.nodes.add('/geographic')
        self.sample(driver=True, wheel_still=True, lidar=True, odom=(0., 0.))
        self.workflow.click('navigation')
        self.assertEqual([key for key, *_ in self.starts], ['driver', 'navigation'])
        self.workflow.click('vision')
        self.assertFalse(driver.stopped)
        self.assertIn('等待导航空闲', self.workflow.messages[-1])

    def test_stop_all_cancels_driver_handoff_before_launching_grasp(self):
        self.workflow.click('driver')
        driver = self.starts[-1][3]
        self.sample(driver=True, wheel_still=True)
        self.workflow.click('vision')
        self.assertTrue(driver.stopped)
        self.workflow.stop_all()
        driver.status = 0
        self.refresh()
        self.assertEqual([key for key, *_ in self.starts], ['driver'])

    def test_air_selection_optional_and_external_executor_exclusion(self):
        self.assertEqual(self.color('air'), 'red')
        self.workflow.click('air', mode='v2')
        self.assertIn('monitor_only:=false', self.starts[-1][1])
        self.assertIn('start_fkie:=false', self.starts[-1][1])
        self.graph.air_mode = 'v2'
        self.sample(v2={'phase': 'IDLE'})
        self.refresh()
        self.assertEqual(self.color('air'), 'yellow')
        self.graph.publishers['/air_ground/v2/uav_state'] = {'/ag_uav_gateway'}
        self.sample(aircraft_v2='/ag_uav_gateway')
        self.refresh()
        self.assertEqual(self.color('air'), 'green')
        self.clock += 3.
        self.sample(v2={'phase': 'IDLE'})
        self.refresh()
        self.assertEqual(self.color('air'), 'yellow')
        self.workflow.stop('air')
        self.assertTrue(self.starts[-1][3].stopped)
        self.starts[-1][3].status = 0
        self.graph.air_mode = 'v1'
        self.sample(v1={'phase': 'IDLE'})
        self.refresh()
        self.workflow.click('air', mode='v2')
        self.assertEqual(len(self.starts), 1)

    def test_v1_green_requires_live_test_topic_evidence_from_a_current_publisher(self):
        self.graph.air_mode = 'v1'
        self.sample(v1={'phase': 'IDLE'}, aircraft_v1='/gone')
        self.graph.publishers['/uav/test'] = {'/uav_link'}
        self.refresh()
        self.assertEqual(self.workflow.states['air'][:2], ('yellow', '车端已启动 · 等飞机'))
        self.sample(aircraft_v1='/uav_link')
        self.refresh()
        self.assertEqual(self.workflow.states['air'][:2], ('green', '收到对端消息'))
        self.clock += 3.
        self.sample(v1={'phase': 'IDLE'}, aircraft_v1_command=True)
        self.refresh()
        self.assertEqual(self.workflow.states['air'][:2], ('green', '收到对端消息'))

    def test_runtime_alone_manages_its_startup_hold(self):
        self.workflow.click('navigation')
        self.prerequisites()
        self.sample(mux=dict(manual_hold=True, teleop_fresh=False, emergency_stop_latched=False,
                             hold_reason='startup', mux_instance_id='mux-a', takeover_generation=0))
        self.refresh()
        self.assertEqual(len(self.starts), 1)
        self.workflow.stop_all()
        self.assertTrue(self.starts[0][3].stopped)

    def test_stop_all_signals_only_owned_groups_in_order(self):
        self.workflow.click('sensors')
        self.workflow.click('driver')
        self.graph.air_mode = 'v1'
        self.refresh()
        self.workflow.stop_all()
        for key in ('driver', 'sensors'):
            active = next(proc for step, _, _, proc in self.starts if step == key)
            self.assertTrue(active.stopped)
            active.status = 0
            self.refresh()
        self.assertEqual(len(self.starts), 2)
