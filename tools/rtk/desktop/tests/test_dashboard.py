import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from dashboard_model import Dashboard, manual_goal_block, split_screen


class DashboardTests(unittest.TestCase):
    def test_split_covers_odd_width_with_monitor_offset(self):
        self.assertEqual(split_screen(-1919, 25, 1919, 1000),
                         ((-1919, 25, 1439, 1000), (-480, 25, 480, 1000)))

    def test_fix_quality_and_expired_sample(self):
        board = Dashboard()
        for number, name in ((4, 'FIXED'), (5, 'FLOAT'), (1, 'SINGLE'), (2, 'DGPS'), (0, 'NO FIX')):
            board.record('gnss', {'gga_quality': str(number), 'satellites': '28'}, 1.)
            self.assertEqual(board.view(1.)['rtk'], f'RTK状态：{name}（卫星：28 颗）')
        self.assertEqual(board.view(5.)['rtk'], 'RTK状态：NO FIX（卫星：—）')

    def test_readiness_requires_registered_map_navigation(self):
        board = Dashboard()
        board.record('geographic', {'state': 'ready', 'registered': True}, 1.)
        board.record('frame', 'odom', 1.)
        self.assertFalse(board.view(1.)['navigation'])
        board.record('frame', 'map', 1.)
        self.assertTrue(board.view(1.)['navigation'])
        board.record('geographic', {'state': 'degraded', 'registered': True}, 1.)
        self.assertTrue(board.view(1.)['navigation'])
        self.assertTrue(board.view(1.)['degraded'])
        self.assertFalse(board.view(5.)['navigation'])

    def test_live_costmap_beats_a_parameter_during_launch_switch(self):
        board = Dashboard()
        board.record('frame', 'odom', 1.)
        board.record('configured_frame', 'map', 2.5)
        self.assertEqual(board.planner_frame(2.5), 'odom')
        self.assertEqual(board.planner_frame(3.5), 'map')

    def test_preparation_shows_manual_goal_and_fault_priority(self):
        board = Dashboard()
        board.record('geographic', {'state': 'collecting', 'registration_samples': 7}, 1.)
        board.record('gnss', {'gga_quality': '4', 'fresh_rtk_usable': 'true'}, 1.)
        board.record('manual', 'RViz手动目标', 1.)
        board.record('mux', {'manual_hold': False, 'emergency_stop_latched': True}, 1.)
        view = board.view(1.)
        self.assertIn('RViz手动目标', view['destination'])
        self.assertIn('7', view['registration'])
        self.assertIn('急停', view['mux'][0])

    def test_point_placeholders_are_literal(self):
        board = Dashboard()
        points = [{'latitude': 20., 'longitude': 120., 'x': .25, 'y': 2.5}] * 3
        board.record('route', {'destination': 'Point1', 'points': points}, 1.)
        self.assertEqual(board.view(1.)['points'], [
            'Point1：20.0000000, 120.0000000 | X: 0.25m, Y: 2.50m',
            'Point2：棍木', 'Point3：棍木',
        ])

    def test_desktop_goal_respects_existing_owners_and_holds(self):
        mux = {'manual_hold': False, 'emergency_stop_latched': False}
        self.assertIsNone(manual_goal_block('odom', True, mux, [], {}, []))
        self.assertIn('优先', manual_goal_block('odom', True, mux, [('external', 1)], {}, []))
        self.assertIn('空地', manual_goal_block('odom', True, mux, [], {},
                                                [{'task_id': 'r', 'phase': 'PAUSED'}]))
        self.assertIn('保持', manual_goal_block('odom', True, {'manual_hold': True}, [], {}, []))
        self.assertIn('配准', manual_goal_block('map', True, mux, [], {'state': 'invalid'}, []))
