"""Resolve the hardware source paired with a vehicle checkout."""
import os
from pathlib import Path


def hardware_root(repository, explicit=None):
    """Use an explicit source, the workspace dependency, or the bundled tree."""
    configured = explicit or os.environ.get('SLICER_HARDWARE_ROOT')
    repository = Path(repository).resolve()
    workspace = repository.parent.parent if repository.parent.name == 'src' else repository
    pinned = workspace / 'deps/slicer_ugv_hardware'
    if configured:
        directory = Path(configured).expanduser()
    elif pinned.exists():
        directory = pinned
    else:
        directory = repository / 'Hardware'
    if not (directory / 'config.yaml').is_file() or not (directory / 'Host').is_dir():
        raise FileNotFoundError(f'Hardware config.yaml and Host/ required at {directory}')
    return directory.resolve()


def hardware_config(hardware, explicit=None):
    """Use the selected site file or the paired hardware checkout's template."""
    selected = explicit or os.environ.get('SLICER_HARDWARE_CONFIG') or Path(hardware) / 'config.yaml'
    path = Path(selected).expanduser()
    if not path.is_file():
        raise FileNotFoundError(f'Hardware configuration file required at {path}')
    return path.resolve()
