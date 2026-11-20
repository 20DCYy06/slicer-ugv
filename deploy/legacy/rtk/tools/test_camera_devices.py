"""Synthetic sysfs/device metadata only; never open cameras or serial ports."""
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parent))
from camera_devices import find_camera_devices


class CameraDevicesTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix='ugv-camera-sysfs-')
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.sys = self.root / 'sys'
        self.dev = self.root / 'dev'
        (self.sys / 'bus/usb/devices').mkdir(parents=True)
        (self.sys / 'class/video4linux').mkdir(parents=True)

    def usb(self, name, serial, bus=2, address=4, product='0b5c', parent=None):
        directory = (parent or self.sys / 'devices/platform/usb2') / name
        directory.mkdir(parents=True)
        for key, value in {'idVendor': '8086', 'idProduct': product,
                           'serial': serial, 'busnum': str(bus), 'devnum': str(address)}.items():
            (directory / key).write_text(value + '\n')
        (self.sys / 'bus/usb/devices' / name).symlink_to(directory)
        node = self.dev / 'bus/usb/{:03d}/{:03d}'.format(bus, address)
        node.parent.mkdir(parents=True, exist_ok=True)
        if not node.exists():
            node.symlink_to('/dev/null')
        return directory, node

    def video(self, usb, number, character=True):
        interface = usb / (usb.name + ':1.0')
        directory = interface / 'video4linux' / ('video' + str(number))
        directory.mkdir(parents=True)
        (directory / 'device').symlink_to(interface)
        (self.sys / 'class/video4linux' / directory.name).symlink_to(directory)
        self.dev.mkdir(exist_ok=True)
        node = self.dev / directory.name
        if character:
            node.symlink_to('/dev/zero')
        else:
            node.write_text('not a character device')
        return node

    def discover(self, serial='TEST_CAMERA'):
        return find_camera_devices(serial, sys_root=self.sys, dev_root=self.dev)

    def test_only_selected_usb_and_its_video_nodes_are_returned(self):
        selected, usb_node = self.usb('2-1.4', 'TEST_CAMERA')
        video10 = self.video(selected, 10)
        video2 = self.video(selected, 2)
        other, _ = self.usb('2-1.5', 'OTHER_CAMERA', address=5)
        self.video(other, 0)
        self.assertEqual(self.discover(), [usb_node, video2, video10])

    def test_matching_serial_on_another_product_is_not_a_d455(self):
        self.usb('2-1.4', 'TEST_CAMERA', product='0b3a')
        with self.assertRaisesRegex(ValueError, 'matched 0'):
            self.discover()

    def test_missing_or_duplicate_matching_device_is_rejected(self):
        with self.assertRaisesRegex(ValueError, 'matched 0'):
            self.discover()
        self.usb('2-1.4', 'TEST_CAMERA')
        self.usb('2-1.5', 'TEST_CAMERA', address=5)
        with self.assertRaisesRegex(ValueError, 'matched 2'):
            self.discover()

    def test_nearest_usb_ancestor_prevents_mapping_another_nested_device(self):
        selected, usb_node = self.usb('2-1.4', 'TEST_CAMERA')
        selected_video = self.video(selected, 1)
        other, _ = self.usb('2-1.4.1', 'OTHER_CAMERA', address=5, parent=selected)
        self.video(other, 3)
        self.assertEqual(self.discover(), [usb_node, selected_video])

    def test_missing_usb_character_node_is_rejected(self):
        _, usb_node = self.usb('2-1.4', 'TEST_CAMERA')
        usb_node.unlink()
        with self.assertRaisesRegex(ValueError, 'character device'):
            self.discover()

    def test_selected_regular_video_file_is_rejected(self):
        selected, _ = self.usb('2-1.4', 'TEST_CAMERA')
        self.video(selected, 1, character=False)
        with self.assertRaisesRegex(ValueError, 'character device'):
            self.discover()

    def test_usb_only_camera_is_allowed_without_mapping_foreign_video_nodes(self):
        _, usb_node = self.usb('2-1.4', 'TEST_CAMERA')
        other, _ = self.usb('2-1.5', 'OTHER_CAMERA', address=5)
        self.video(other, 0)
        self.assertEqual(self.discover(), [usb_node])

    def test_empty_serial_is_rejected(self):
        with self.assertRaisesRegex(ValueError, 'serial'):
            self.discover('')


if __name__ == '__main__':
    unittest.main()
