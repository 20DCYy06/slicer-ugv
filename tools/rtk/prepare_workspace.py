#!/usr/bin/env python3
"""Generate the RTK install configuration from the vehicle and sensor profiles."""

import argparse
import importlib.util
from pathlib import Path
import sys

import yaml

REPOSITORY = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPOSITORY / 'tools'))
from hardware_path import hardware_config, hardware_root
SOURCE_CONFIG = REPOSITORY / 'src/ugv_bringup/config'
RUNTIME_GENERATOR = (
    REPOSITORY / 'src/slicer_navigation_without_mapping/scripts/prepare_runtime.py'
)


def generate(config_dir, hardware=None, host_config=None):
    hardware = hardware_root(REPOSITORY, hardware)
    host_config = hardware_config(hardware, host_config)
    spec = importlib.util.spec_from_file_location('base_runtime', RUNTIME_GENERATOR)
    runtime = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(runtime)
    navigation = runtime.generate(runtime.CONFIG_ROOT, config_dir / 'online_runtime',
                                  REPOSITORY, hardware, host_config)
    # The RTK Guard caps the DWA output independently of the no-RTK template.
    field = yaml.safe_load((SOURCE_CONFIG / 'dwa_field.yaml').read_text())['DWAPlannerROS']
    guard_path = config_dir / 'online_runtime/config/slicer_config.yaml'
    guard = yaml.safe_load(guard_path.read_text())
    guard['navigation']['max_forward'] = field['max_vel_x']
    guard['navigation']['accel_linear'] = field['acc_lim_x']
    guard_path.write_text(yaml.safe_dump(guard, sort_keys=False))
    footprint = navigation['local_costmap']['footprint']
    params = {
        'global_costmap': {'footprint': footprint, 'footprint_padding': 0.0},
        'local_costmap': {'footprint': footprint, 'footprint_padding': 0.0},
        'TebLocalPlannerROS': {'footprint_model': {'type': 'polygon', 'vertices': footprint}},
    }
    (config_dir / 'footprint.yaml').write_text(yaml.safe_dump(params, sort_keys=False))

    driver_name = 'slicer_driver_body_speed.yaml'
    driver = yaml.safe_load((SOURCE_CONFIG / driver_name).read_text())
    vehicle = yaml.safe_load(host_config.read_text())
    serial = vehicle['serial']
    driver.update(
        serial_port=serial['port'],
        serial_number=serial['serial_number'],
        pwm_signs=serial['pwm_signs'],
        encoder_signs=serial['encoder_signs'],
        counts_per_revolution=serial['counts_per_revolution'],
        wheel_diameter=0.065,
        track_width=0.179,
        wheelbase=0.13,
        mcu_pwm_accept_max_permille=vehicle['control'].get('mcu_pwm_accept_max_permille', 800),
    )
    (config_dir / driver_name).write_text(yaml.safe_dump(driver, sort_keys=False))
    print('RTK_WORKSPACE2_PARAMETERS_READY board=' + serial['serial_number'])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config-dir', type=Path, required=True, help='Generated config output')
    parser.add_argument('--hardware-root', type=Path, help='Hardware checkout containing config.yaml and Host/')
    parser.add_argument('--hardware-config', type=Path, help='Site hardware configuration file')
    args = parser.parse_args()
    generate(args.config_dir, args.hardware_root, args.hardware_config)


if __name__ == '__main__':
    main()
