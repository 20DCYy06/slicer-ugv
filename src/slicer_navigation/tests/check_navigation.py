#!/usr/bin/env python3
"""仅在 --network none 的 Noetic 容器中检查；不映射串口和激光设备。"""
import ast
import os
from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET
import yaml

package = Path(__file__).resolve().parents[1]
if os.environ.get('ROS_DISTRO') != 'noetic':
    raise RuntimeError('This check requires ROS 1 Noetic')
root = package.parents[2]
config = yaml.safe_load((root / 'config/slicer_config.yaml').read_text(encoding='utf-8'))
generated_nav = yaml.safe_load((root / '.generated/navigation.yaml').read_text(encoding='utf-8'))
fusion = yaml.safe_load((root / '.generated/fusion.yaml').read_text(encoding='utf-8'))
teb = generated_nav['TebLocalPlannerROS']
teb_override = yaml.safe_load((package / 'config/teb_local_planner_params.yaml').read_text(encoding='utf-8'))['TebLocalPlannerROS']
teb = dict(teb, **teb_override)
assert teb_override['allow_init_with_backwards_motion'] is False
assert teb_override['feasibility_check_no_poses'] == 3
assert teb_override['shrink_horizon_backup'] is True
assert config['navigation']['max_reverse'] == 0.0
assert teb['max_vel_x_backwards'] == .02  # Valid TEB soft bound; outlet enforces body vx >= 0.
assert teb['min_turning_radius'] == config['navigation']['min_turning_radius'] == 0.0
assert teb['max_vel_theta'] == config['navigation']['max_angular']
assert 0 < config['navigation']['turn_start_angular'] <= teb['max_vel_theta']
assert teb['acc_lim_theta'] == config['navigation']['accel_angular'] == 1.2
assert teb['weight_kinematics_forward_drive'] == config['navigation']['teb_forward_drive_weight'] == 900.
assert teb['min_obstacle_dist'] == config['navigation']['min_obstacle_dist'] == .144
assert teb['enable_homotopy_class_planning'] is False
assert teb['max_number_classes'] == 2
assert teb['global_plan_overwrite_orientation'] is True
assert teb['penalty_epsilon'] < teb['max_vel_x_backwards']
assert teb['inflation_dist'] == .25 and teb['weight_obstacle'] == 60.
assert fusion['odom1'] == '/wheel/odom' and fusion['odom1_config'] == [i == 6 for i in range(15)]
assert fusion['publish_tf'] is True and fusion['two_d_mode'] is True
assert fusion['odom0'] == '/lio/odom' and fusion['odom0_config'][5]
assert generated_nav['local_costmap']['publish_frequency'] == config['navigation']['local_costmap_publish_hz'] == 4.0
assert generated_nav['local_costmap']['inflation_layer']['inflation_radius'] == config['navigation']['inflation_radius'] == .192
assert generated_nav['global_costmap']['inflation_layer']['inflation_radius'] == .192
assert generated_nav['global_costmap']['inflation_layer']['cost_scaling_factor'] == 4.5
assert generated_nav['global_costmap']['static_layer']['lethal_cost_threshold'] == 100
footprint = generated_nav['local_costmap']['footprint']
assert min(p[0] for p in footprint) <= -.145
expected_front = max(config['chassis']['body_length']/2, config['navigation']['travel_envelope']['front_x'])+config['navigation']['footprint_padding']
assert abs(max(p[0] for p in footprint)-expected_front) < 1e-9
assert abs(max(abs(p[1]) for p in footprint) - .14) < 1e-9
assert teb['footprint_model']['vertices'] == generated_nav['global_costmap']['footprint'] == footprint
for path in package.rglob('*.py'):
    ast.parse(path.read_text(encoding='utf-8'), filename=str(path))
for path in list(package.rglob('*.yaml')) + list(package.rglob('*.rviz')):
    yaml.safe_load(path.read_text(encoding='utf-8'))
for path in list(package.rglob('*.launch')) + [package / 'package.xml']:
    ET.parse(str(path))
patrol_launch = ET.parse(str(package / 'launch/patrol.launch')).getroot()
patrol_node = patrol_launch.find("node[@name='slicer_multi_goal_patrol']")
assert patrol_node is not None and patrol_node.get('required') != 'true'
assert patrol_launch.find("arg[@name='auto_exit']") is not None
for path in package.glob('scripts/*.sh'):
    subprocess.run(['bash', '-n', str(path)], check=True)
print('STATIC_CHECK_PASS', flush=True)
subprocess.run(['rosrun','slicer_navigation','slicer_bspline_check'],check=True)
subprocess.run([sys.executable, str(package / 'scripts/runtime_preflight.py')], check=True)
subprocess.run([sys.executable, str(package / 'tests/test_wasd_panel.py')], check=True,
               env=dict(os.environ, QT_QPA_PLATFORM='offscreen'))
subprocess.run([sys.executable, str(package / 'tests/test_core.py'), '-v'], check=True)
subprocess.run([sys.executable, str(package / 'tests/test_arm_cloud_filter.py'), '-v'], check=True)
subprocess.run([sys.executable, str(package / 'tests/test_patrol_flow.py'), '-v'], check=True)
subprocess.run([sys.executable, str(package / 'tests/ros_patrol_smoke.py')], check=True)
subprocess.run([sys.executable, str(package / 'tests/ros_smoke.py'), '--nav-only'], check=True,
               env=dict(os.environ,SLICER_BSPLINE_EVIDENCE=str(root/'logs/navigation/bspline_validation.json')))
print('NAVIGATION_CHECK_PASS', flush=True)
