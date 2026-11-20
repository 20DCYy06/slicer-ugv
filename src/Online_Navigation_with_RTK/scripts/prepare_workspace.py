#!/usr/bin/env python3
import argparse
import importlib.util
from pathlib import Path
import yaml
root=Path(__file__).resolve().parents[1]
source=root.parent/'Online_Navigation_Without_RTK'
workspace2=root.parents[1]
spec=importlib.util.spec_from_file_location('base_runtime',source/'catkin_ws/src/slicer_navigation_without_mapping/scripts/prepare_runtime.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
source_config=root/'src/ugv_bringup/config'
parser=argparse.ArgumentParser()
parser.add_argument('--config-dir', type=Path, default=source_config)
args=parser.parse_args()
config=args.config_dir
runtime=config/'online_runtime'
nav=module.generate(source,runtime,workspace2)
footprint=nav['local_costmap']['footprint']
params={'global_costmap':{'footprint':footprint,'footprint_padding':0.},'local_costmap':{'footprint':footprint,'footprint_padding':0.},'TebLocalPlannerROS':{'footprint_model':{'type':'polygon','vertices':footprint}}}
(config/'footprint.yaml').write_text(yaml.safe_dump(params,sort_keys=False))
driver_name='slicer_driver_body_speed.yaml'
driver=yaml.safe_load((source_config/driver_name).read_text())
host=yaml.safe_load((workspace2/'Hardware/config.yaml').read_text());serial=host['serial']
driver.update(serial_number=serial['serial_number'],pwm_signs=serial['pwm_signs'],encoder_signs=serial['encoder_signs'],counts_per_revolution=serial['counts_per_revolution'],wheel_diameter=.065,track_width=.179,wheelbase=.13,motor_output_limit_percent=host['control']['motor_output_limit_percent'],mcu_pwm_accept_max_permille=800)
(config/driver_name).write_text(yaml.safe_dump(driver,sort_keys=False))
print('RTK_WORKSPACE2_PARAMETERS_READY board='+serial['serial_number'])
