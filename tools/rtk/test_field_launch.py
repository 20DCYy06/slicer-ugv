"""Resolve operator presets with Noetic XML; no nodes/devices are started."""
import math
from pathlib import Path
import sys
import unittest

import roslaunch
import rospkg
import yaml


ROOT = Path(__file__).resolve().parents[2]
LAUNCH = ROOT / 'src/ugv_bringup/launch'
CONFIG = ROOT / 'src/ugv_bringup/config'
sys.path.insert(0, str(ROOT / 'src/ugv_slicer_control/scripts'))
from navigation_core import planner_limits


def load(name, arguments=()):
    config = roslaunch.config.ROSLaunchConfig()
    roslaunch.xmlloader.XmlLoader().load(str(LAUNCH / name), config,
                                       argv=list(arguments), verbose=False)
    if config.config_errors:
        raise AssertionError(config.config_errors)
    return config


class FieldLaunchTests(unittest.TestCase):
    def test_outdoor_rviz_has_no_default_map_or_camera_dependency(self):
        import yaml
        manager = yaml.safe_load((ROOT / 'src/ugv_bringup/rviz/outdoor_navigation.rviz').read_text())['Visualization Manager']
        self.assertEqual(manager['Global Options']['Fixed Frame'], 'map')
        displays = {item.get('Topic'): item for item in manager['Displays']}
        for topic in ('/map', '/online_depth/obstacles', '/online_depth/clearing'):
            self.assertFalse(displays[topic]['Enabled'])
        self.assertTrue(displays['/scan']['Enabled'])
        self.assertTrue(displays['/ugv/move_base/global_costmap/costmap']['Enabled'])

    def test_rtk_receiver_and_local_sensor_steps_start_disjoint_nodes(self):
        rtk = load('slicer_field_sensors.launch', ['start_local_sensors:=false'])
        local = load('slicer_field_sensors.launch', ['geographic:=false'])
        rtk_nodes = {(n.package, n.type) for n in rtk.nodes}
        local_nodes = {(n.package, n.type) for n in local.nodes}
        self.assertTrue(any(package == 'cup_gnss' for package, _ in rtk_nodes))
        self.assertFalse(any(package == 'cup_gnss' for package, _ in local_nodes))
        self.assertFalse(any('fastlio' in kind for _, kind in rtk_nodes))
        self.assertTrue(any('fastlio' in kind for _, kind in local_nodes))

    def test_body_shortcut_selects_only_0x16_driver_without_tf(self):
        config = load('slicer_body_driver.launch')
        self.assertEqual([(n.package, n.type) for n in config.nodes],
                         [('ugv_slicer_control', 'c30d_driver_node.py')])
        self.assertEqual(config.params['/slicer_c30d_driver/control_mode'].value, 'body_speed')
        self.assertFalse(config.params['/slicer_c30d_driver/publish_tf'].value)
        bringup = Path(rospkg.RosPack().get_path('ugv_bringup'))
        self.assertEqual(Path(config.params['/slicer_c30d_driver/config_file'].value),
                         bringup / 'config/slicer_driver_body_speed.yaml')

    def test_sensor_shortcut_retains_single_local_ekf_and_tuned_wz(self):
        config = load('slicer_field_sensors.launch', ['geographic:=false'])
        self.assertEqual(sum(n.type == 'ekf_localization_node' for n in config.nodes), 1)
        self.assertFalse(any(n.type in ('c30d_driver_node.py', 'move_base', 'slam_gmapping')
                             for n in config.nodes))
        self.assertEqual(config.params['/slicer_ekf/world_frame'].value, 'odom')
        self.assertEqual(config.params['/slicer_ekf/process_noise_covariance'].value[176], 0.2)

    def test_default_sensors_add_global_estimator_without_navigation_or_driver(self):
        config = load('slicer_field_sensors.launch')
        self.assertEqual(sum(n.type == 'ekf_localization_node' for n in config.nodes), 2)
        self.assertEqual(config.params['/slicer_ekf/world_frame'].value, 'odom')
        self.assertEqual(config.params['/global_localization/ekf/world_frame'].value, 'map')
        self.assertFalse(any(n.type in ('c30d_driver_node.py', 'move_base', 'slam_gmapping')
                             for n in config.nodes))

    def assert_navigation_chain(self, config, field=True, heading=False):
        types = [n.type for n in config.nodes]
        for name in ('move_base', 'arc_velocity_guard.py', 'velocity_mux.py'):
            self.assertEqual(types.count(name), 1)
        for name in ('c30d_driver_node.py', 'ekf_localization_node', 'goal.py', 'control.py'):
            self.assertNotIn(name, types)
        params = config.params
        plugin = params['/ugv/move_base/base_local_planner'].value
        expected = 'slicer_navigation/DWAPlannerROS' if heading else 'dwa_local_planner/DWAPlannerROS'
        self.assertEqual(plugin, expected)
        prefix = '/ugv/move_base/DWAPlannerROS/'
        planner = {key[len(prefix):]: param.value for key, param in params.items() if key.startswith(prefix)}
        for key in ('max_vel_x', 'max_vel_trans'):
            self.assertEqual(planner[key], .8 if field else .4)
        self.assertAlmostEqual(planner['max_vel_theta'], math.pi / 4 if field else .25)
        if field:
            self.assertEqual((planner['acc_lim_x'], planner['acc_lim_trans'], planner['sim_time']),
                             (1.0, 1.0, 1.05))
            source = Path(params['/slicer_nav_arc_guard/config_file'].value)
            source_config = yaml.safe_load(source.read_text())
            effective = planner_limits(source_config['navigation'], plugin, planner)
            self.assertEqual(effective['max_forward'], .8)
            self.assertEqual(effective['accel_linear'], 1.0)
            self.assertAlmostEqual(effective['max_angular'], math.pi / 4)
            timeout = params['/slicer_nav_arc_guard/command_timeout'].value
            speed = effective['max_forward']
            brake = min(effective['brake_linear'], effective['accel_linear'])
            self.assertEqual(brake, .2)
            travel = speed * (speed / brake + timeout)
            local = '/ugv/move_base/local_costmap/'
            radius = max(math.hypot(*vertex) for vertex in params[local + 'footprint'].value)
            grid_period = (1 / params[local + 'update_frequency'].value
                           + 1 / params[local + 'publish_frequency'].value)
            # Allow one cell for rounding and nominal update/publication lag.
            required_radius = travel + radius + params[local + 'resolution'].value + speed * grid_period
            for axis in ('width', 'height'):
                self.assertGreater(params[local + axis].value / 2, required_radius)
            laser_x, laser_y, _ = source_config['sensors']['base_to_laser']['xyz']
            scan = local + 'obstacle_layer/scan/'
            self.assertGreaterEqual(params[scan + 'raytrace_range'].value,
                                    required_radius + math.hypot(laser_x, laser_y))
            self.assertGreaterEqual(params[scan + 'obstacle_range'].value,
                                    params[scan + 'raytrace_range'].value)
        self.assertEqual(params['/ugv/move_base/DWAPlannerROS/min_vel_x'].value, 0.0)
        self.assertFalse(params['/ugv/move_base/recovery_behavior_enabled'].value)
        self.assertEqual(params['/ugv/move_base/recovery_behaviors'].value, [])
        self.assertEqual(params['/slicer_nav_arc_guard/input_topic'].value, '/cmd_vel/nav_raw')

    def assert_no_depth(self, config):
        self.assertFalse(any(n.type == 'depth_obstacles.py' or
                             n.package == 'realsense2_camera' or
                             'realsense2_camera/RealSenseNodeFactory' in n.args
                             for n in config.nodes))
        for scope in ('global_costmap', 'local_costmap'):
            prefix = '/ugv/move_base/' + scope + '/'
            layers = config.params[prefix + 'plugins'].value
            self.assertNotIn('depth_layer', [layer['name'] for layer in layers])
            self.assertFalse(any('depth_layer/' in key for key in config.params
                                 if key.startswith(prefix)))
            self.assertEqual(config.params[prefix + 'obstacle_layer/observation_sources'].value,
                             'scan')
            self.assertTrue(config.params[prefix + 'obstacle_layer/scan/marking'].value)
            self.assertTrue(config.params[prefix + 'obstacle_layer/scan/clearing'].value)

    def test_default_geographic_profile_uses_fixed_map_grid_without_static_or_depth(self):
        config = load('slicer_field_navigation.launch')
        self.assert_navigation_chain(config, heading=True)
        self.assert_no_depth(config)
        self.assertIn('/ugv/move_base/', config.clear_params)
        self.assertFalse(any(n.package in ('gmapping', 'realsense2_camera') for n in config.nodes))
        p = config.params
        prefix = '/ugv/move_base/'
        self.assertEqual(p[prefix + 'global_costmap/global_frame'].value, 'map')
        self.assertFalse(p[prefix + 'global_costmap/rolling_window'].value)
        self.assertFalse(p[prefix + 'global_costmap/static_map'].value)
        self.assertEqual(p[prefix + 'global_costmap/width'].value, 90.0)
        self.assertEqual(p[prefix + 'global_costmap/height'].value, 90.0)
        self.assertEqual(p[prefix + 'global_costmap/origin_x'].value, -45.0)
        self.assertEqual(p[prefix + 'global_costmap/origin_y'].value, -45.0)
        self.assertEqual(p[prefix + 'global_costmap/resolution'].value, .05)
        self.assertNotIn('static_layer', [layer['name'] for layer in
                         p[prefix + 'global_costmap/plugins'].value])
        self.assertTrue(p[prefix + 'GlobalPlanner/allow_unknown'].value)
        self.assertEqual(p[prefix + 'local_costmap/global_frame'].value, 'odom')
        self.assertTrue(p[prefix + 'local_costmap/rolling_window'].value)
        self.assertTrue(p[prefix + 'local_costmap/track_unknown_space'].value)
        self.assertEqual(p[prefix + 'base_global_planner'].value, 'slicer_navigation/BSplineGlobalPlanner')
        self.assertTrue(p['/slicer_nav_arc_guard/adaptive_forward_speed'].value)
        self.assertEqual(p[prefix + 'GlobalPlanner/lethal_cost'].value, 175)
        self.assertFalse(p[prefix + 'GlobalPlanner/use_grid_path'].value)
        self.assertEqual(p[prefix + 'GlobalPlanner/smoothing/max_cost'].value, 173)
        self.assertTrue(p[prefix + 'GlobalPlanner/smoothing/observed_prefix'].value)
        self.assertEqual(p[prefix + 'DWAPlannerROS/vx_samples'].value, 9)
        self.assertEqual(p[prefix + 'DWAPlannerROS/start_heading_rate'].value, .45)
        self.assertEqual(p[prefix + 'DWAPlannerROS/start_heading_lookahead'].value, .20)
        self.assertEqual(p[prefix + 'DWAPlannerROS/start_heading_kp'].value, 1.6)
        self.assertEqual(p[prefix + 'GlobalPlanner/continuity/preferred_clearance'].value, .45)
        self.assertEqual(p[prefix + 'DWAPlannerROS/vth_samples'].value, 21)
        self.assertFalse(p['/slicer_nav_arc_guard/registration_reverse'].value)
        for scope in ('global_costmap', 'local_costmap'):
            costmap = prefix + scope + '/'
            self.assertEqual(p[costmap + 'inflation_layer/inflation_radius'].value, .25)
            self.assertEqual(p[costmap + 'obstacle_layer/scan/obstacle_range'].value, 4.5)
            self.assertEqual(p[costmap + 'obstacle_layer/scan/raytrace_range'].value, 4.5)
            self.assertFalse(p[costmap + 'obstacle_layer/scan/inf_is_valid'].value)
        self.assertTrue(p[prefix + 'DWAPlannerROS/start_heading_enabled'].value)

    def test_heading_can_be_disabled_without_switching_local_planner(self):
        p = load('slicer_field_navigation.launch', ['start_heading_enabled:=false']).params
        prefix = '/ugv/move_base/'
        self.assertFalse(p[prefix + 'DWAPlannerROS/start_heading_enabled'].value)
        self.assertEqual(p[prefix + 'base_local_planner'].value, 'slicer_navigation/DWAPlannerROS')


    def test_preparation_uses_separate_speed_profile_without_adaptive_guard(self):
        p = load('slicer_field_navigation.launch', ['geographic:=false', 'preparation:=true']).params
        prefix = '/ugv/move_base/DWAPlannerROS/'
        for key in ('max_vel_x', 'max_vel_trans'):
            self.assertEqual(p[prefix + key].value, .35)
        for key in ('acc_lim_x', 'acc_lim_trans'):
            self.assertEqual(p[prefix + key].value, .30)
        self.assertEqual(p[prefix + 'sim_time'].value, 1.7)
        self.assertFalse(p['/slicer_nav_arc_guard/adaptive_forward_speed'].value)
        self.assertTrue(p['/slicer_nav_arc_guard/registration_reverse'].value)
        self.assertEqual(p['/ugv/move_base/base_local_planner'].value,
                         'dwa_local_planner/DWAPlannerROS')

    def test_bootstrap_odom_profile_remains_explicit(self):
        config = load('slicer_field_navigation.launch', ['geographic:=false'])
        self.assert_navigation_chain(config, heading=True)
        self.assert_no_depth(config)
        p = config.params
        self.assertEqual(p['/ugv/move_base/global_costmap/global_frame'].value, 'odom')
        self.assertTrue(p['/ugv/move_base/global_costmap/rolling_window'].value)
        self.assertEqual(p['/ugv/move_base/global_costmap/width'].value, 24.0)
        self.assertEqual(p['/ugv/move_base/global_costmap/height'].value, 16.0)

    def test_online_profile_keeps_map_layers_and_one_gmapping_without_duplicate_ekf(self):
        config = load('slicer_field_navigation.launch', ['mapping:=true'])
        self.assert_navigation_chain(config, field=False)
        self.assert_no_depth(config)
        self.assertEqual(sum(n.type == 'slam_gmapping' for n in config.nodes), 1)
        self.assertEqual(config.params['/ugv/move_base/global_costmap/global_frame'].value, 'map')
        self.assertFalse(config.params['/ugv/move_base/global_costmap/rolling_window'].value)
        self.assertTrue(config.params['/ugv/move_base/global_costmap/static_layer/subscribe_to_updates'].value)
        self.assertEqual(config.params['/ugv/move_base/local_costmap/width'].value, 4.0)
        self.assertEqual(config.params['/ugv/move_base/local_costmap/height'].value, 4.0)

    def test_explicit_depth_starts_processor_and_camera_without_static_map_dependency(self):
        config = load('slicer_field_navigation.launch', ['enable_depth:=true'])
        self.assertEqual(sum(n.type == 'depth_obstacles.py' for n in config.nodes), 1)
        self.assertEqual(sum('realsense2_camera/RealSenseNodeFactory' in n.args
                             for n in config.nodes), 1)
        layers = config.params['/ugv/move_base/global_costmap/plugins'].value
        self.assertEqual([layer['name'] for layer in layers],
                         ['obstacle_layer', 'depth_layer', 'inflation_layer'])

    def test_existing_camera_can_be_reused_without_starting_another_driver(self):
        config = load('slicer_field_navigation.launch',
                      ['mapping:=true', 'enable_depth:=true', 'start_camera:=false'])
        self.assertFalse(any('realsense2_camera/RealSenseNodeFactory' in n.args
                             for n in config.nodes))
        self.assertTrue(any(n.type == 'depth_obstacles.py' for n in config.nodes))

    def test_explicit_cruise_selection_remains_available_for_both_map_modes(self):
        for mode in ('false', 'true'):
            baseline = load('slicer_field_navigation.launch', ['mapping:=' + mode])
            for filename, cap in (('dwa_cruise_020.yaml', .20), ('dwa_cruise_030.yaml', .30)):
                selected = load('slicer_field_navigation.launch', ['mapping:=' + mode,
                                'cruise_config:=' + str(CONFIG / filename)])
                before = {key: p.value for key, p in baseline.params.items()}
                after = {key: p.value for key, p in selected.params.items()}
                changed = {key for key in before.keys() | after.keys()
                           if before.get(key) != after.get(key)}
                expected = {'/ugv/move_base/DWAPlannerROS/max_vel_x',
                            '/ugv/move_base/DWAPlannerROS/max_vel_trans'}
                if mode == 'false':
                    expected.add('/ugv/move_base/DWAPlannerROS/max_vel_theta')
                self.assertEqual(changed, expected)
                self.assertEqual(after['/ugv/move_base/DWAPlannerROS/max_vel_x'], cap)


if __name__ == '__main__':
    unittest.main()
