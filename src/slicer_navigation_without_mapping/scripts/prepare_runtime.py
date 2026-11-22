#!/usr/bin/env python3
"""从原车现场配置生成本包独立运行参数，不修改原导航包或其.generated。"""
import argparse
import json
import os
from pathlib import Path
import sys

import yaml

SCRIPT = Path(__file__).resolve()
REPO = SCRIPT.parents[3]
TOOLS = SCRIPT.parent if (SCRIPT.parent / 'hardware_path.py').is_file() else REPO / 'tools'
sys.path.insert(0, str(TOOLS))
from hardware_path import hardware_config, hardware_root
BRINGUP_CONFIG = REPO / 'src/ugv_bringup/config'
if not BRINGUP_CONFIG.is_dir():
    BRINGUP_CONFIG = Path(__file__).resolve().parents[2] / 'share/ugv_bringup/config'
CONFIG_ROOT = BRINGUP_CONFIG / 'no_rtk'


def generate(root, output, workspace2=None, hardware=None, host_config=None):
    root=Path(root);output=Path(output);g=output/'.generated';g.mkdir(parents=True,exist_ok=True)
    (output/'config').mkdir(exist_ok=True)
    c=yaml.safe_load((root/'slicer_config.yaml').read_text());s=c['sensors'];n=c['navigation'];v=c['chassis']
    def save(name,value):(g/name).write_text(yaml.safe_dump(value,sort_keys=False),encoding='utf-8')
    (output/'config/slicer_config.yaml').write_text(yaml.safe_dump(c,sort_keys=False),encoding='utf-8')
    nav=yaml.safe_load((root/'templates/navigation.yaml').read_text())
    pad=n['footprint_padding'];envelope=n['travel_envelope']
    rear=min(-v['body_length']/2,envelope['rear_x'])-pad;front=max(v['body_length']/2,envelope['front_x'])+pad
    width=max(v['body_width']/2,envelope['half_width'])+pad
    footprint=[[rear,-width],[rear,width],[front,width],[front,-width]]
    for name in ('global_costmap','local_costmap'):
        costmap=nav[name];costmap.update(footprint=footprint,footprint_padding=0.0,robot_base_frame=c['frames']['base'])
        costmap['inflation_layer'].update(inflation_radius=n['inflation_radius'],cost_scaling_factor=n['cost_scaling_factor'])
        costmap['obstacle_layer']['scan'].update(min_obstacle_height=n['min_obstacle_height'],max_obstacle_height=s['scan_max_height'])
    teb=nav['TebLocalPlannerROS'];teb.update(max_vel_x=n['max_forward'],max_vel_x_backwards=max(.02,n['max_reverse']),
        max_vel_theta=n['max_angular'],acc_lim_x=n['accel_linear'],acc_lim_theta=n['accel_angular'],
        min_turning_radius=n['min_turning_radius'],xy_goal_tolerance=n['xy_tolerance'],yaw_goal_tolerance=n['yaw_tolerance'],
        weight_kinematics_forward_drive=n['teb_forward_drive_weight'],min_obstacle_dist=n['min_obstacle_dist'])
    teb['footprint_model']={'type':'polygon','vertices':footprint}
    nav.update(controller_frequency=n['controller_hz'],planner_frequency=n['planner_hz'])
    teb['wheelbase']=v['wheelbase']
    save('vehicle.yaml',nav)
    for name in ('sensor_launch.yaml','fusion.yaml'):
        save(name,yaml.safe_load((root/'templates'/name).read_text()))
    lio=yaml.safe_load((root/'templates/lio.yaml').read_text())
    lio['mapping'].update(extrinsic_T=s['lidar_to_imu_translation'],extrinsic_R=s['lidar_to_imu_rotation']);lio['publish']['tf_en']=False
    save('lio.yaml',lio)
    livox=json.loads((root/'templates/MID360.json').read_text());(g/'MID360.json').write_text(json.dumps(livox,indent=2))
    scan=yaml.safe_load((root/'templates/scan.yaml').read_text());scan.update(min_height=s['scan_min_height'],max_height=s['scan_max_height'],target_frame=c['frames']['base'])
    save('scan.yaml',scan)
    slam=yaml.safe_load((root/'templates/gmapping.yaml').read_text())
    slam.update(map_frame=c['frames']['map'],odom_frame=c['frames']['odom'],base_frame=c['frames']['base'])
    save('slam.yaml',slam)
    # Snapshot the selected host configuration alongside this runtime's ROS parameters.
    repository = workspace2 or os.environ.get('WORKSPACE2_ROOT') or REPO
    host_yaml = hardware_config(hardware_root(repository, hardware), host_config).read_bytes()
    host = yaml.safe_load(host_yaml)
    (output / 'config/c30d_host.yaml').write_bytes(host_yaml)
    drive=yaml.safe_load((BRINGUP_CONFIG/'slicer_driver_body_speed.yaml').read_text())
    serial=host['serial']
    control=dict(control_mode='body_speed',base_frame=c['frames']['base'],odom_frame=c['frames']['odom'],
        publish_tf=False,output_topic='/cmd_vel',nav_topic='/cmd_vel/nav',mission_topic='/cmd_vel/mission',
        teleop_topic='/teleop/cmd_vel',odom_topic='/wheel/odom',publish_hz=20.,command_timeout_s=.25,
        telemetry_timeout_s=host['control']['telemetry_timeout_s'],max_linear=n['max_forward'],
        motor_output_limit_percent=drive['motor_output_limit_percent'],
        max_angular=n['max_angular'],wheel_diameter=v['wheel_diameter'],track_width=v['track_width'],
        wheelbase=v['wheelbase'],counts_per_revolution=serial['counts_per_revolution'],
        encoder_signs=serial['encoder_signs'],serial_number=serial['serial_number'],
        wheel_ports=v.get('wheel_ports'))
    (output/'config/control.yaml').write_text(yaml.safe_dump(control,sort_keys=False))
    print('ONLINE_RUNTIME',output,'footprint=',footprint,'max_v=',n['max_forward'],'max_w=',n['max_angular'],flush=True)
    return nav


if __name__=='__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=CONFIG_ROOT, help='Source NoRTK configuration directory')
    parser.add_argument('--output', type=Path, required=True, help='Generated runtime directory')
    parser.add_argument('--workspace2', type=Path, help='Vehicle checkout, or workspace with deps/')
    parser.add_argument('--hardware-root', type=Path, help='Hardware checkout containing config.yaml and Host/')
    parser.add_argument('--hardware-config', type=Path, help='Site hardware configuration file')
    args = parser.parse_args()
    generate(args.root, args.output, args.workspace2, args.hardware_root, args.hardware_config)
