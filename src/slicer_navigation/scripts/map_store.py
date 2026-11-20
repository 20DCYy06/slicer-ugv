#!/usr/bin/env python3
"""先完整生成地图，再备份并替换同名地图；失败恢复旧文件。"""
import datetime
import fcntl
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tempfile
import yaml


def commit_map(staged, directory, name):
    staged, directory = Path(staged), Path(directory)
    metadata = yaml.safe_load(staged.with_suffix('.yaml').read_text(encoding='utf-8'))
    if not isinstance(metadata, dict) or float(metadata['resolution']) <= 0 or len(metadata['origin']) != 3:
        raise ValueError('Invalid generated map YAML')
    if not staged.with_suffix('.pgm').is_file() or staged.with_suffix('.pgm').stat().st_size == 0:
        raise ValueError('Generated map image is empty')
    metadata['image'] = name + '.pgm'  # 使用相对路径，不能保留临时目录路径。
    staged.with_suffix('.yaml').write_text(yaml.safe_dump(metadata, sort_keys=False), encoding='utf-8')
    for suffix in ('.pgm', '.yaml'):
        with open(staged.with_suffix(suffix), 'rb') as handle:
            os.fsync(handle.fileno())
    old = [suffix for suffix in ('.pgm', '.yaml') if (directory / (name + suffix)).exists()]
    backup = None
    if old:
        now = datetime.datetime.now(datetime.timezone(datetime.timedelta(hours=8)))
        backup = directory / '.backups' / (name + '_' + now.strftime('%Y%m%d_%H%M%S_%f'))
        backup.mkdir(parents=True)
        for suffix in old:
            shutil.copy2(directory / (name + suffix), backup / (name + suffix))
    changed = []
    try:
        for suffix in ('.pgm', '.yaml'):
            os.replace(str(staged.with_suffix(suffix)), str(directory / (name + suffix)))
            changed.append(suffix)
    except OSError:
        for suffix in changed:
            destination = directory / (name + suffix)
            if suffix in old:
                shutil.copy2(backup / (name + suffix), destination)
            else:
                destination.unlink()
        raise
    return backup


def main():
    directory, name = Path(sys.argv[1]).expanduser().resolve(), sys.argv[2]
    if not re.fullmatch(r'[A-Za-z0-9_-]+', name):
        raise ValueError('Invalid map name')
    directory.mkdir(parents=True, exist_ok=True)
    # 同名并发保存串行化；新图生成失败时旧图仍完好。
    with open(directory / ('.' + name + '.save.lock'), 'a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        with tempfile.TemporaryDirectory(prefix='.' + name + '.saving-', dir=str(directory)) as stage:
            staged = Path(stage) / name
            subprocess.run(['rosrun', 'map_server', 'map_saver', '-f', str(staged), 'map:=/map'],
                           check=True, timeout=30)
            backup = commit_map(staged, directory, name)
    if backup:
        print('PREVIOUS_MAP_BACKUP: ' + str(backup), flush=True)
    print('MAP_SAVED: ' + str(directory / (name + '.yaml')), flush=True)


if __name__ == '__main__':
    try:
        main()
    except Exception as exc:
        print('MAP_SAVE_FAILED: ' + str(exc), file=sys.stderr)
        sys.exit(1)
