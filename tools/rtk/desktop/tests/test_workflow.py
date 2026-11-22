"""Mock the desktop process runner and the global ROS graph."""
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from dashboard_model import Dashboard
from workflow import Workflow
from workflow_discovery import Graph


class Process:
    def __init__(self):
        self.status = None
        self.stopped = False

    def poll(self):
        return self.status

    def stop(self):
        self.stopped = True

    def lines(self):
        return []


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

    def refresh(self, vision=False):
        self.workflow.update(self.graph, vision=vision)

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
        for key in ('sensors', 'driver', 'odom'):
            self.assertEqual(self.workflow.states[key][:2], ('green', '外部运行中'))
            self.workflow.click(key)
        self.assertEqual(self.starts, [])
        self.clock += 10.
        self.refresh()
        self.assertEqual(self.color('odom'), 'yellow')
        self.workflow.click('odom')
        self.assertEqual(self.starts, [])

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
        self.assertEqual(self.starts[-1][1][-1], 'slicer_field_navigation.launch')
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
        self.workflow.local_scan = lambda: {'vision': {820}}
        self.workflow.click('vision')
        self.assertEqual(self.starts, [])
        self.assertIn('外部终端', self.workflow.messages[-1])

    def test_visual_environment_and_owned_stop(self):
        self.workflow.click('vision')
        key, argv, env, proc = self.starts[-1]
        self.assertEqual(key, 'vision')
        self.assertEqual(list(argv), ['rosrun', 'slicer_visual_grasp', 'run.sh', '--run'])
        for name in ('DISPLAY', 'XAUTHORITY', 'ROS_MASTER_URI', 'ROS_IP'):
            self.assertEqual(env[name], self.env[name])
        self.assertEqual(env['VISUAL_GRASP_DISPLAY'], self.env['DISPLAY'])
        self.assertNotIn('ROS_NAMESPACE', env)
        self.refresh(vision=True)
        self.assertEqual(self.color('vision'), 'green')
        self.workflow.click('vision')
        self.assertTrue(proc.stopped)

    def test_air_selection_optional_and_external_executor_exclusion(self):
        self.assertEqual(self.color('air'), 'red')
        self.workflow.click('air', mode='v2', monitor=False)
        self.assertIn('monitor_only:=false', self.starts[-1][1])
        self.assertIn('start_fkie:=false', self.starts[-1][1])
        self.graph.air_mode = 'v2'
        self.sample(v2={'phase': 'IDLE'})
        self.refresh()
        self.assertEqual(self.color('air'), 'green')
        self.workflow.stop('air')
        self.assertTrue(self.starts[-1][3].stopped)
        self.starts[-1][3].status = 0
        self.graph.air_mode = 'v1'
        self.sample(v1={'phase': 'IDLE'})
        self.refresh()
        self.workflow.click('air', mode='v2')
        self.assertEqual(len(self.starts), 1)

    def test_stop_all_signals_only_owned_groups_in_order(self):
        self.workflow.click('sensors')
        self.workflow.click('driver')
        self.workflow.click('vision')
        self.graph.air_mode = 'v1'
        self.refresh()
        self.workflow.stop_all()
        for key in ('vision', 'driver', 'sensors'):
            active = next(proc for step, _, _, proc in self.starts if step == key)
            self.assertTrue(active.stopped)
            active.status = 0
            self.refresh()
        self.assertEqual(len(self.starts), 3)
