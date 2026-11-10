#!/usr/bin/env python3
"""Read sysfs to select one D455's device nodes; never open a device."""
import argparse
from pathlib import Path
import sys


def find_camera_devices(serial, sys_root=Path('/sys'), dev_root=Path('/dev')):
    """Return USB then numeric-sorted video paths for one 8086:0b5c serial.

    Alternate roots are for synthetic metadata tests, not public CLI options.
    Metadata checks follow symlinks but never read from character devices.
    """
    if not serial or serial != serial.strip():
        raise ValueError('camera serial must be nonempty and have no surrounding spaces')
    usb_directory = Path(sys_root) / 'bus/usb/devices'
    matches = set()
    for entry in usb_directory.iterdir() if usb_directory.is_dir() else ():
        try:
            identity = tuple((entry / key).read_text().strip()
                             for key in ('idVendor', 'idProduct', 'serial'))
        except (OSError, UnicodeError):
            continue
        if identity == ('8086', '0b5c', serial):
            matches.add(entry.resolve())
    if len(matches) != 1:
        raise ValueError('D455 serial {} matched {} USB devices'.format(serial, len(matches)))
    usb = matches.pop()
    bus, address = (int((usb / key).read_text().strip()) for key in ('busnum', 'devnum'))
    if not (1 <= bus <= 999 and 1 <= address <= 999):
        raise ValueError('invalid D455 USB bus/device number')
    nodes = [Path(dev_root) / 'bus/usb/{:03d}/{:03d}'.format(bus, address)]
    video_directory = Path(sys_root) / 'class/video4linux'
    entries = video_directory.iterdir() if video_directory.is_dir() else ()
    for entry in sorted((p for p in entries if p.name.startswith('video')
                         and p.name[5:].isdigit()), key=lambda p: int(p.name[5:])):
        try:
            device = (entry / 'device').resolve(strict=True)
        except OSError:
            continue
        # Associate with the nearest USB device, not a matching path prefix or
        # another video device nested under the same upstream hub.
        for ancestor in (device,) + tuple(device.parents):
            if (ancestor / 'idVendor').is_file() and (ancestor / 'idProduct').is_file():
                if ancestor == usb:
                    nodes.append(Path(dev_root) / entry.name)
                break
    for node in nodes:
        if not node.is_char_device():
            raise ValueError('selected camera node is not an existing character device: ' + str(node))
    return nodes


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('serial', help='exact USB serial of the D455 to expose')
    args = parser.parse_args(argv)
    try:
        nodes = find_camera_devices(args.serial)
    except (OSError, ValueError) as error:
        print('camera: ' + str(error), file=sys.stderr)
        return 2
    for node in nodes:
        print(node)
    return 0


if __name__ == '__main__':
    sys.exit(main())
