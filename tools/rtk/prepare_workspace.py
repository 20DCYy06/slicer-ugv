#!/usr/bin/env python3
"""Generate the RTK install configuration from the vehicle and sensor profiles."""

import argparse
import importlib.util
from pathlib import Path

import yaml

REPOSITORY = Path(__file__).resolve().parents[2]
SOURCE_CONFIG = REPOSITORY / 'src/ugv_bringup/config'
RUNTIME_GENERATOR = (
    REPOSITORY / 'src/slicer_navigation_without_mapping/scripts/prepare_runtime.py'
)


def generate(config_dir):
    spec = importlib.util.spec_from_file_location('base_runtime', RUNTIME_GENERATOR)
    runtime = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(runtime)
    navigation = runtime.generate(runtime.CONFIG_ROOT, config_dir / 'online_runtime', REPOSITORY)
    footprint = navigation['local_costmap']['footprint']
    params = {
        'global_costmap': {'footprint': footprint, 'footprint_padding': 0.0},
        'local_costmap': {'footprint': footprint, 'footprint_padding': 0.0},
        'TebLocalPlannerROS': {'footprint_model': {'type': 'polygon', 'vertices': footprint}},
    }
    (config_dir / 'footprint.yaml').write_text(yaml.safe_dump(params, sort_keys=False))

    driver_name = 'slicer_driver_body_speed.yaml'
    driver = yaml.safe_load((SOURCE_CONFIG / driver_name).read_text())
    vehicle = yaml.safe_load((REPOSITORY / 'Hardware/config.yaml').read_text())
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
        motor_output_limit_percent=vehicle['control']['motor_output_limit_percent'],
        mcu_pwm_accept_max_permille=vehicle['control'].get('mcu_pwm_accept_max_permille', 800),
    )
    (config_dir / driver_name).write_text(yaml.safe_dump(driver, sort_keys=False))
    print('RTK_WORKSPACE2_PARAMETERS_READY board=' + serial['serial_number'])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config-dir', type=Path, required=True, help='Generated config output')
    generate(parser.parse_args().config_dir)


if __name__ == '__main__':
    main()
