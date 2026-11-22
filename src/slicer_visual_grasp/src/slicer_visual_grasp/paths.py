"""Configuration paths shared by source and Catkin install layouts."""
from pathlib import Path
import sys

PACKAGE_ROOT = Path(__file__).resolve().parents[2]
if not (PACKAGE_ROOT / 'config/config.json').is_file():
    from rospkg import RosPack
    PACKAGE_ROOT = Path(RosPack().get_path('slicer_visual_grasp'))

DEFAULT_CONFIG = PACKAGE_ROOT / 'config/config.json'
SUPPORT_ROOT = PACKAGE_ROOT / 'support'
if SUPPORT_ROOT.is_dir():
    HARDWARE = SUPPORT_ROOT / 'Hardware'
    HOST_CONFIG = HARDWARE / 'config.yaml'
else:
    SUPPORT_ROOT = PACKAGE_ROOT.parents[1]
    sys.path.insert(0, str(SUPPORT_ROOT / 'tools'))
    from hardware_path import hardware_config, hardware_root

    HARDWARE = hardware_root(SUPPORT_ROOT)
    HOST_CONFIG = hardware_config(HARDWARE)
MANUAL = SUPPORT_ROOT / 'tools/manual'
DEVICE_CONFIGS = {
    'hardware_config': HOST_CONFIG,
    'airgrip_config': MANUAL / 'airgrip/config.json',
    'servo_config': MANUAL / 'b15_servo/config.json',
}


def device_config(config, root, key):
    if key in config:
        return root / config[key]
    return DEVICE_CONFIGS[key]
