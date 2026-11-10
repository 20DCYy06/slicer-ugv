#!/usr/bin/env python3
import math
from pathlib import Path
import sys
import tempfile
import unittest
from unittest import mock
import yaml
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from navigation_core import (append_waypoint, load_waypoints, validate_waypoints,
                             limit_twist, swept_clear, footprint_clear, parse_footprint,
                             forward_obstacle_distance, obstacle_speed_scale, slew_arc,
                             project_safe_twist, front_scan_clearance, limit_front_approach)
from map_store import commit_map


class NavigationTests(unittest.TestCase):
    def test_xy_waypoint_can_omit_yaw(self):
        data = validate_waypoints({'frame_id': 'map', 'waypoints': [
            {'name': 'B', 'x': 1.0, 'y': 2.0},
            {'name': 'C', 'x': 3.0, 'y': 4.0, 'yaw': 1.2}]})
        self.assertNotIn('yaw', data['waypoints'][0])
        self.assertEqual(data['waypoints'][1]['yaw'], 1.2)

    def test_null_waypoints_and_name_first(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'points.yaml'
            path.write_text('frame_id: map\nwaypoints:\n', encoding='utf-8')
            self.assertEqual(load_waypoints(path)['waypoints'], [])
            for i in range(5):
                name, count = append_waypoint(path, 'map', dict(x=i, y=-i, yaw=.2))
                self.assertEqual(count, i+1)
            data = load_waypoints(path)
            self.assertTrue(all(next(iter(p)) == 'name' for p in data['waypoints']))
            self.assertIn('- name: point_001\n  x:', path.read_text())
            # 单独 `-` 产生空占位；录点时安全清理、备份，并保留全部真实坐标。
            data['waypoints'].insert(1, None)
            path.write_text(yaml.safe_dump(data, sort_keys=False))
            before_placeholder = path.read_bytes()
            name, count = append_waypoint(path, 'map', dict(x=9, y=8, yaw=.3))
            self.assertEqual(count, 6)
            self.assertEqual(load_waypoints(path)['waypoints'][0]['x'], 0)
            backups = list((path.parent / '.backups').glob('*.bak'))
            self.assertEqual(backups[0].read_bytes(), before_placeholder)
            path.write_text('frame_id: map\nwaypoints: broken\n')
            before = path.read_bytes()
            with self.assertRaises(ValueError):
                append_waypoint(path, 'map', dict(x=0, y=0, yaw=0))
            self.assertEqual(path.read_bytes(), before)

    def test_same_name_map_replace_and_rollback(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);stage=root/'stage';stage.mkdir()
            prefix=stage/'room'
            def generated(content):
                prefix.with_suffix('.pgm').write_bytes(content)
                prefix.with_suffix('.yaml').write_text(yaml.safe_dump(dict(
                    image=str(prefix.with_suffix('.pgm')),resolution=.05,origin=[0,0,0])))
            generated(b'old-map')
            self.assertIsNone(commit_map(prefix,root,'room'))
            generated(b'new-map')
            backup=commit_map(prefix,root,'room')
            self.assertEqual((root/'room.pgm').read_bytes(),b'new-map')
            self.assertEqual((backup/'room.pgm').read_bytes(),b'old-map')
            self.assertEqual(yaml.safe_load((root/'room.yaml').read_text())['image'],'room.pgm')
            generated(b'failed-map')
            import os
            actual_replace=os.replace
            def replace(source,destination):
                if str(destination).endswith('.yaml'):raise OSError('simulated commit failure')
                return actual_replace(source,destination)
            with mock.patch('map_store.os.replace',side_effect=replace):
                with self.assertRaises(OSError):commit_map(prefix,root,'room')
            self.assertEqual((root/'room.pgm').read_bytes(),b'new-map')
            self.assertEqual(yaml.safe_load((root/'room.yaml').read_text())['image'],'room.pgm')

    def test_atomic_append_and_invalid_target(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'waypoints.yaml'
            for i in range(3):
                append_waypoint(path, 'map', dict(x=i, y=1, yaw=0.5))
            data = load_waypoints(path)
            self.assertEqual(len(data['waypoints']), 3)
            original = path.read_bytes()
            with self.assertRaises(ValueError):
                append_waypoint(path, 'map', dict(x=float('nan'), y=0, yaw=0))
            self.assertEqual(original, path.read_bytes())
            with self.assertRaises(ValueError):
                append_waypoint(path, 'odom', dict(x=0, y=0, yaw=0))

    def test_differential_drive_allows_pure_spin(self):
        self.assertEqual(limit_twist(0, .704, .125, .04, .72, .01), (0, .704))
        self.assertEqual(limit_twist(0, -.704, .125, .04, .72, .01), (0, -.704))
        self.assertEqual(limit_twist(0, 0, .125, .04, .72, .01), (0, 0))
        self.assertEqual(limit_twist(-.3, 0, .125, .04, .72, .01), (-.04, 0))
        self.assertEqual(limit_twist(.3, 3, .125, .04, .72, .01), (.125, .72))

    def test_near_obstacle_slowdown_and_arc_slew(self):
        grid = dict(width=80, height=80, resolution=.025, origin=(-1,-1,0), data=[0]*6400)
        footprint = [(-.165,-.14), (-.165,.14), (.165,.14), (.165,-.14)]
        # 正前方 x=0.4m，距离已加宽的前沿约 0.23m；背后障碍不影响前进减速。
        grid['data'][40*80+56] = 100
        distance = forward_obstacle_distance((0,0,0), footprint, grid, .1)
        self.assertTrue(.20 < distance < .25, distance)
        factor = obstacle_speed_scale(distance, .12, .45, .25)
        self.assertTrue(.25 < factor < 1)
        self.assertEqual(obstacle_speed_scale(.1, .12, .45, .25), 0)
        self.assertEqual(obstacle_speed_scale(.5, .12, .45, .25), 1)
        self.assertEqual(forward_obstacle_distance((0,0,0), footprint, grid, -.1), math.inf)
        grid['data'][40*80+56] = 0
        grid['data'][44*80+48] = 100
        self.assertTrue(math.isfinite(forward_obstacle_distance((0,0,0), footprint, grid, .1)))
        self.assertEqual(forward_obstacle_distance((0,0,0), footprint, grid, .1,
                         front_half_angle=.1), math.inf)
        v,w = slew_arc((0.,0.), (.09,.75), .1, .2, .45)
        self.assertAlmostEqual(v, .02)
        self.assertLessEqual(w, .0450001)
        self.assertEqual(slew_arc((v,w), (0.,0.), .1, .2, .45), (0.,0.))
        reversed_v, reversed_w = slew_arc((v,w), (.09,-.75), .1, .2, .45)
        self.assertAlmostEqual(reversed_v, .04)
        self.assertAlmostEqual(reversed_w, 0.)

    def test_front_stop_uses_laser_origin_and_keeps_pure_spin(self):
        # 同一面墙：base_link 扫描 range=.22，雷达原点前置.07，因此实际间距=.15。
        base_gap = front_scan_clearance([.22], 0., .01, .05, 20., (0.,0.,0.), .07, .14)
        laser_gap = front_scan_clearance([.15], 0., .01, .05, 20., (.07,0.,0.), .07, .14)
        self.assertAlmostEqual(base_gap,.15)
        self.assertAlmostEqual(laser_gap,.15)
        self.assertEqual(limit_front_approach((.125,.3), .125, base_gap,.15,.2,.15), (0.,0.))
        self.assertEqual(limit_front_approach((0.,.7), 0., .10,.15,.2,.15), (0.,.7))
        self.assertEqual(limit_front_approach((-.10,0.), 0., .10,.15,.2,.15), (-.10,0.))
        slower = limit_front_approach((.125,.3), .125, .20,.15,.2,.15)
        self.assertTrue(0 < slower[0] < .125)
        self.assertAlmostEqual(slower[1]/slower[0], .3/.125)
        self.assertEqual(limit_front_approach((.125,0.), .125, math.inf,.15,.2,.15), (.125,0.))
        # 侧面/后面/无效回波不能触发前向停车。
        for angle in (math.pi, math.pi/2):
            self.assertEqual(front_scan_clearance([.15], angle,.01,.05,20.,(0.,0.,0.),.07,.14),math.inf)
        self.assertEqual(front_scan_clearance([math.nan,math.inf],0.,.1,.05,20.,(0.,0.,0.),.07,.14),math.inf)

    def test_swept_collision_and_unknown(self):
        grid = dict(width=80, height=80, resolution=.025, origin=(-1,-1,0), data=[0]*6400)
        footprint = [(-.135,-.11), (-.135,.11), (.135,.11), (.135,-.11)]
        self.assertEqual(parse_footprint('[[-0.135,-0.11],[-0.135,0.11],[0.135,0.11],[0.135,-0.11]]'), footprint)
        self.assertTrue(swept_clear((0,0,0), .2, .4, footprint, grid, 1))
        # 障碍格位于起点足迹外，却会被未来圆弧扫过。
        grid['data'][43*80+51] = 100
        self.assertTrue(footprint_clear((0,0,0), footprint, grid))
        self.assertFalse(swept_clear((0,0,0), .2, .4, footprint, grid, 1))

    def test_guard_projects_arc_without_blindly_zeroing(self):
        grid = dict(width=80, height=80, resolution=.025, origin=(-1,-1,0), data=[0]*6400)
        footprint = [(-.135,-.11),(-.135,.11),(.135,.11),(.135,-.11)]
        grid['data'][40*80+58] = 100
        v, w, note = project_safe_twist((0,0,0), .4, 0, footprint, grid, 1)
        self.assertTrue(0 < v < .4, (v, note))
        self.assertEqual(w, 0)
        self.assertTrue(swept_clear((0,0,0), v, w, footprint, grid, 1))
        grid['data'][40*80+40] = 100
        self.assertEqual(project_safe_twist((0,0,0), .4, 0, footprint, grid, 1)[:2], (0,0))
        grid['data'][40*80+40] = 0
        self.assertEqual(project_safe_twist((0,0,0), 0, .2, footprint, grid, 1)[:2], (0,.2))
        grid['data'][43*80+51] = -1
        self.assertFalse(swept_clear((0,0,0), .2, .4, footprint, grid, 1))
        self.assertTrue(swept_clear((0,0,0), .2, .4, footprint, grid, 1, unknown=False))
        self.assertFalse(footprint_clear((.95,0,0), footprint, grid))
        grid['origin'] = (-1, -1, math.pi/2)
        self.assertTrue(footprint_clear((-1.5,-.5,math.pi/2), footprint, grid))


if __name__ == '__main__':
    unittest.main()
