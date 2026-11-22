"""Mock the desktop process runner and the global ROS graph."""
from pathlib import Path
import sys
import unittest
from unittest.mock import Mock, patch

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
        with patch('workflow.time.monotonic', return_value=28.), patch('workflow.os.killpg') as kill:
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
        self.sample(lidar=True, odom=(0., 0.), driver=True, action=[],
                    odom_still=True, geographic={'state': 'invalid'},
                    mux={'manual_hold': held, 'teleop_fresh': False, 'emergency_stop_latched': False},
                    quality={'gga_quality': 4 if fixed else 5, 'fresh_rtk_usable': str(fixed).lower()})
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
        self.workflow.click('odom')
        self.assertEqual(self.starts[-1][1][-1], 'geographic:=false')
        self.graph.frame = 'odom'
        self.graph.subscribers['/ugv/move_base/goal'] = {'/ugv/move_base'}
        self.sample(action=[])
        self.refresh()
        self.assertEqual(self.color('odom'), 'green')
        self.workflow.click('odom')
        self.starts[-1][3].status = 0
        self.graph.frame = None
        self.graph.subscribers.clear()
        self.refresh()
        self.assertEqual(self.color('odom'), 'red')

    def test_external_ready_and_stale_owner_both_block_duplicate(self):
        self.prerequisites()
        expected = {'sensors': '已就绪', 'driver': '已完成', 'odom': '已就绪'}
        for key in ('sensors', 'driver', 'odom'):
            self.assertEqual(self.workflow.states[key][:2], ('green', expected[key]))
            self.workflow.click(key)
        self.assertEqual(self.starts, [])
        self.clock += 10.
        self.refresh()
        self.assertEqual(self.color('odom'), 'yellow')
        self.workflow.click('odom')
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
        self.workflow.click('odom')
        self.assertEqual(self.workflow.states['odom'][:2], ('yellow', '等待本地里程计'))
        self.starts[-1][3].status = 0
        self.refresh()
        self.assertEqual(self.workflow.states['odom'][:2], ('red', '进程已退出'))
        self.workflow.click('odom')
        self.assertEqual([key for key, *_ in self.starts], ['odom', 'odom'])

    def test_calibration_release_then_initialize_then_restore_hold(self):
        self.prerequisites(fixed=False)
        self.assertIsNone(self.workflow.click('calibrate'))
        self.assertEqual(self.starts, [])
        self.prerequisites()
        self.assertEqual(self.workflow.click('calibrate'), 'confirm')
        self.workflow.click('calibrate', confirmed=True)
        self.assertIn('release', self.starts[-1][1])
        self.assertEqual(self.color('calibrate'), 'yellow')
        self.starts[-1][3].status = 0
        self.sample(mux={'manual_hold': False, 'teleop_fresh': False, 'emergency_stop_latched': False})
        self.refresh()
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

    def test_map_waits_only_for_owned_odom_and_keeps_sensors_and_driver(self):
        self.workflow.click('sensors')
        self.workflow.click('driver')
        self.workflow.click('odom')
        odom = self.starts[-1][3]
        self.prerequisites()
        self.sample(geographic={'state': 'ready', 'registered': True})
        self.refresh()
        self.workflow.click('map')
        self.assertTrue(odom.stopped)
        self.assertEqual(self.color('map'), 'yellow')
        self.assertTrue(all(not p.stopped for _, _, _, p in self.starts[:2]))
        odom.status = 0
        self.refresh()
        self.assertEqual(len(self.starts), 3)
        self.graph.frame = None
        self.graph.nodes.discard('/ugv/move_base')
        self.graph.subscribers.clear()
        self.refresh()
        self.assertEqual(Path(self.starts[-1][1][-1]).name, 'slicer_field_navigation.launch')
        self.assertEqual(self.color('odom'), 'red')

    def test_map_refuses_to_stop_external_odom(self):
        self.prerequisites()
        self.sample(geographic={'state': 'ready', 'registered': True})
        self.refresh()
        self.workflow.click('map')
        self.assertEqual(self.starts, [])
        self.assertIn('外部终端', self.workflow.messages[-1])

    def test_map_starts_after_external_terminal_stops_odom_and_mux(self):
        self.prerequisites()
        self.graph.frame = None
        self.graph.nodes.discard('/ugv/move_base')
        self.graph.subscribers.clear()
        self.sample(mux={}, geographic={'state': 'ready', 'registered': True})
        self.refresh()
        self.workflow.click('map')
        self.refresh()
        self.assertEqual([step for step, *_ in self.starts], ['map'])

    def test_stop_all_cancels_pending_map_even_when_no_process_is_owned(self):
        self.prerequisites()
        self.graph.frame = None
        self.graph.nodes.discard('/ugv/move_base')
        self.graph.subscribers.clear()
        self.sample(geographic={'state': 'ready', 'registered': True})
        self.refresh()
        self.workflow.click('map')
        self.assertTrue(self.workflow.switching)
        self.workflow.stop_all()
        self.refresh()
        self.assertEqual(self.starts, [])

    def test_calibration_can_retry_when_manager_remains_collecting_but_worker_exited(self):
        self.prerequisites()
        self.sample(geographic={'state': 'collecting'})
        self.graph.nodes.add('/slicer_geographic_client_123')
        self.refresh()
        self.assertEqual(self.color('calibrate'), 'yellow')
        self.assertIsNone(self.workflow.click('calibrate'))
        self.graph.nodes.discard('/slicer_geographic_client_123')
        self.refresh()
        self.assertEqual(self.color('calibrate'), 'red')
        self.assertEqual(self.workflow.click('calibrate'), 'confirm')

    def test_stop_all_holds_owned_navigation_before_signalling_it(self):
        self.workflow.click('odom')
        navigation = self.starts[-1][3]
        self.prerequisites(held=False)
        self.workflow.stop_all()
        self.assertFalse(navigation.stopped)
        self.assertIn('hold', self.starts[-1][1])
        self.starts[-1][3].status = 0
        self.refresh()
        self.assertTrue(navigation.stopped)

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
        self.assertEqual(Path(argv[0]).name, 'run.sh')
        self.assertEqual(argv[1:], ('--headless', '--pick-on-start'))
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
        self.assertEqual(self.starts[-1][1][1:], ('--headless',))
        another.log_lines = ['GRASP_STATE recovery']
        self.refresh()
        self.assertEqual(another.sent, ['release'])

    def test_grasp_does_not_start_under_existing_serial_owner(self):
        self.graph.publishers['/wheel/odom'] = {'/slicer_c30d_driver'}
        self.workflow.click('vision')
        self.assertEqual(self.starts, [])
        self.assertEqual(self.workflow.states['vision'][:2], ('yellow', '请先停止导航和底盘'))
        self.graph.publishers.clear()
        self.workflow.click('vision')
        self.assertEqual(self.starts[-1][0], 'vision')

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

    def test_owned_idle_navigation_releases_only_generation_bound_startup_hold(self):
        self.workflow.click('odom')
        self.prerequisites()
        mux = dict(manual_hold=True, teleop_fresh=False, emergency_stop_latched=False,
                   hold_reason='manual_takeover', mux_instance_id='mux-a', takeover_generation=3)
        self.sample(mux=mux)
        self.refresh()
        self.assertEqual(len(self.starts), 1)
        self.sample(mux=dict(mux, hold_reason='startup', emergency_stop_latched=True))
        self.refresh()
        self.assertEqual(len(self.starts), 1)
        self.sample(mux=dict(mux, hold_reason='startup'))
        self.refresh()
        key, argv, _, proc = self.starts[-1]
        self.assertEqual(key, 'startup_release')
        self.assertEqual(argv[-4:], ('--expected-mux-instance', 'mux-a', '--expected-generation', '3'))
        self.workflow.stop_all()
        self.assertTrue(proc.stopped)
        proc.status = -2
        self.refresh()
        # The cached Mux state may still be startup-held while release is in flight.
        self.assertEqual(self.starts[-1][0], 'stop_nav_hold')
        self.assertFalse(self.starts[0][3].stopped)

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
