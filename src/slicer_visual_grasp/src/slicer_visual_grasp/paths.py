"""Configuration paths shared by source and Catkin install layouts."""
from pathlib import Path

PACKAGE_ROOT = Path(__file__).resolve().parents[2]
if not (PACKAGE_ROOT / 'config/config.json').is_file():
    from rospkg import RosPack
    PACKAGE_ROOT = Path(RosPack().get_path('slicer_visual_grasp'))

DEFAULT_CONFIG = PACKAGE_ROOT / 'config/config.json'
SUPPORT_ROOT = PACKAGE_ROOT / 'support'
if not SUPPORT_ROOT.is_dir():
    SUPPORT_ROOT = PACKAGE_ROOT.parents[1]

HARDWARE = SUPPORT_ROOT / 'Hardware'
MANUAL = SUPPORT_ROOT / 'tools/manual'
DEVICE_CONFIGS = {
    'hardware_config': HARDWARE / 'config.yaml',
    'airgrip_config': MANUAL / 'airgrip/config.json',
}


def device_config(config, root, key):
    if key in config:
        return root / config[key]
    return DEVICE_CONFIGS[key]
