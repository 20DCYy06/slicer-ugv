#!/usr/bin/env python3
"""Shell argv checks only: fake Docker, no containers, ROS or serial I/O.

/dev/null supplies a stat-able character device for the option test. It is not
a C30D candidate, and neither this test nor fake Docker opens the device.
"""
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


SHELL = Path(__file__).resolve().with_name('slicer_shell.sh')


class SlicerShellTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix='ugv-shell-test-')
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.argv_file = self.root / 'docker-argv.json'
        self.character_alias = self.root / 'character-device'
        self.character_alias.symlink_to('/dev/null')
        fake_docker = self.root / 'docker'
        fake_docker.write_text(
            '#!%s\n' % sys.executable
            + 'import json, os, sys\n'
            + 'with open(os.environ["UGV_TEST_DOCKER_ARGV"], "w") as out:\n'
            + '    json.dump(sys.argv[1:], out)\n', encoding='utf-8')
        fake_docker.chmod(0o755)
        self.env = dict(os.environ)
        self.env.pop('UGV_SERIAL_DEVICE', None)
        self.env.pop('UGV_GNSS_DEVICE', None)
        self.env.pop('UGV_NTRIP_CREDENTIALS', None)
        self.env.pop('UGV_CAMERA_SERIAL', None)
        self.env.pop('UGV_RUNTIME_CONFIG', None)
        self.env.update(PATH=str(self.root) + os.pathsep + self.env['PATH'],
                        UGV_TEST_DOCKER_ARGV=str(self.argv_file),
                        UGV_SLICER_ROOT='/unused/read only slicer',
                        UGV_NOETIC_IMAGE='ugv-test:no-container')

    def run_shell(self, *command, device=None, stdin=subprocess.PIPE):
        env = dict(self.env)
        if device is not None:
            env['UGV_SERIAL_DEVICE'] = device
        return subprocess.run(['bash', str(SHELL), *command], env=env,
                              stdin=stdin, capture_output=True,
                              text=True, timeout=10)

    def docker_args(self):
        return json.loads(self.argv_file.read_text(encoding='utf-8'))

    def test_default_shell_has_no_device_mapping(self):
        result = self.run_shell()
        self.assertEqual(result.returncode, 0, result.stderr)
        args = self.docker_args()
        self.assertFalse(any(arg.startswith('--device') for arg in args))
        self.assertFalse(any(arg.startswith('--group-add') for arg in args))
        self.assertNotIn('--privileged', args)
        self.assertEqual(args[-2:], ['bash', 'bash'])
        self.assertIn('type=bind,src=/unused/read only slicer,dst=/slicer,readonly', args)

    def test_explicit_character_device_adds_only_that_device_and_its_gid(self):
        result = self.run_shell(device='/dev/null')
        self.assertEqual(result.returncode, 0, result.stderr)
        args = self.docker_args()
        self.assertEqual(args.count('--device'), 1)
        self.assertEqual(args[args.index('--device') + 1], '/dev/null')
        self.assertEqual(args.count('--group-add'), 1)
        self.assertEqual(args[args.index('--group-add') + 1], str(os.stat('/dev/null').st_gid))
        self.assertNotIn('--privileged', args)
        self.assertEqual(args[-2:], ['bash', 'bash'])

    def test_character_device_alias_maps_resolved_path_for_driver_auto_discovery(self):
        # /dev/stdin follows this child's /dev/null stdin, without creating
        # devices or touching a serial port. Like /dev/c30d, it is an alias.
        result = self.run_shell(device='/dev/stdin', stdin=subprocess.DEVNULL)
        self.assertEqual(result.returncode, 0, result.stderr)
        args = self.docker_args()
        self.assertEqual(args[args.index('--device') + 1], '/dev/null')
        self.assertEqual(args.count('--device'), 1)
        self.assertEqual(args[args.index('--group-add') + 1], str(os.stat('/dev/null').st_gid))

    def test_missing_device_is_rejected_before_docker(self):
        result = self.run_shell(device='/dev/ugv-missing-' + self.root.name)
        self.assertNotEqual(result.returncode, 0)
        self.assertFalse(self.argv_file.exists())
        self.assertIn('UGV_SERIAL_DEVICE', result.stderr)

    def test_regular_file_is_rejected_before_docker(self):
        regular_file = self.root / 'not-a-device'
        regular_file.write_text('fixture', encoding='utf-8')
        result = self.run_shell(device=str(regular_file))
        self.assertNotEqual(result.returncode, 0)
        self.assertFalse(self.argv_file.exists())

    def test_character_device_outside_dev_is_rejected_before_docker(self):
        result = self.run_shell(device=str(self.character_alias))
        self.assertNotEqual(result.returncode, 0)
        self.assertFalse(self.argv_file.exists())

    def test_dev_path_cannot_traverse_to_a_character_device_outside_dev(self):
        result = self.run_shell(device='/dev/..' + str(self.character_alias))
        self.assertNotEqual(result.returncode, 0)
        self.assertFalse(self.argv_file.exists())

    def test_command_arguments_with_spaces_are_preserved(self):
        command = ('printf', '%s\\n', 'argument with spaces', '', '$(not executed)')
        result = self.run_shell(*command)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(self.docker_args()[-len(command):], list(command))

    def test_gnss_device_has_a_stable_alias_without_mapping_the_chassis(self):
        self.env['UGV_GNSS_DEVICE'] = '/dev/null'
        result = self.run_shell()
        self.assertEqual(result.returncode, 0, result.stderr)
        args = self.docker_args()
        self.assertEqual(args.count('--device'), 1)
        self.assertEqual(args[args.index('--device') + 1], '/dev/null:/dev/gnss')

    def test_missing_gnss_device_is_rejected_before_docker(self):
        self.env['UGV_GNSS_DEVICE'] = '/dev/gnss-TEST_ONLY-missing'
        result = self.run_shell()
        self.assertNotEqual(result.returncode, 0)
        self.assertFalse(self.argv_file.exists())

    def test_credentials_are_mounted_read_only_without_becoming_ros_arguments(self):
        fixture = self.root / 'TEST_ONLY.json'
        fixture.write_text('{}')
        self.env['UGV_NTRIP_CREDENTIALS'] = str(fixture)
        result = self.run_shell('roslaunch', 'ugv_bringup', 'slicer_gnss.launch')
        self.assertEqual(result.returncode, 0, result.stderr)
        args = self.docker_args()
        self.assertIn('type=bind,src=' + str(fixture) +
                      ',dst=/run/secrets/slicer_ntrip.json,readonly', args)
        self.assertFalse(any(x.startswith('--device') for x in args))

    def test_chassis_and_gnss_cannot_map_the_same_physical_device_twice(self):
        self.env['UGV_GNSS_DEVICE'] = '/dev/null'
        result = self.run_shell(device='/dev/null')
        self.assertNotEqual(result.returncode, 0)
        self.assertFalse(self.argv_file.exists())

    def test_runtime_file_provides_image_and_network_without_mapping_devices(self):
        config = self.root / 'runtime.env'
        config.write_text('UGV_RUNTIME_IMAGE=TEST_ONLY:runtime\n'
                          'UGV_RUNTIME_MASTER_URI=http://127.0.0.1:11359\n'
                          'UGV_RUNTIME_IP=127.0.0.2\n')
        self.env['UGV_RUNTIME_CONFIG'] = str(config)
        for key in ('UGV_NOETIC_IMAGE', 'ROS_MASTER_URI', 'ROS_IP'):
            self.env.pop(key, None)
        result = self.run_shell()
        self.assertEqual(result.returncode, 0, result.stderr)
        args = self.docker_args()
        self.assertIn('TEST_ONLY:runtime', args)
        self.assertIn('ROS_MASTER_URI=http://127.0.0.1:11359', args)
        self.assertIn('ROS_IP=127.0.0.2', args)
        self.assertNotIn('--device', args)

    def test_explicit_environment_overrides_runtime_file(self):
        config = self.root / 'runtime.env'
        config.write_text('UGV_RUNTIME_IMAGE=TEST_ONLY:default\n'
                          'UGV_RUNTIME_MASTER_URI=http://127.0.0.1:11359\n'
                          'UGV_RUNTIME_IP=127.0.0.2\n')
        self.env.update(UGV_RUNTIME_CONFIG=str(config),
                        ROS_MASTER_URI='http://127.0.0.1:11360', ROS_IP='127.0.0.3')
        result = self.run_shell()
        self.assertEqual(result.returncode, 0, result.stderr)
        args = self.docker_args()
        self.assertIn('ugv-test:no-container', args)
        self.assertIn('ROS_MASTER_URI=http://127.0.0.1:11360', args)
        self.assertIn('ROS_IP=127.0.0.3', args)
        self.assertNotIn('TEST_ONLY:default', args)

    def camera_discovery_stub(self, devices=('/dev/null', '/dev/zero'), exit_code=0):
        fake_python = self.root / 'python3'
        fake_python.write_text(
            '#!%s\n' % sys.executable
            + 'import json, os, sys\n'
            + 'assert sys.argv[1].endswith("/tools/camera_devices.py"), sys.argv\n'
            + 'assert sys.argv[2:] == ["--", "TEST_CAMERA"], sys.argv\n'
            + 'for path in json.loads(os.environ["UGV_TEST_CAMERA_DEVICES"]): print(path)\n'
            + 'raise SystemExit(int(os.environ["UGV_TEST_CAMERA_EXIT"]))\n')
        fake_python.chmod(0o755)
        self.env.update(UGV_CAMERA_SERIAL='TEST_CAMERA',
                        UGV_TEST_CAMERA_DEVICES=json.dumps(devices),
                        UGV_TEST_CAMERA_EXIT=str(exit_code))

    def test_camera_opt_in_maps_only_the_selected_character_nodes(self):
        self.camera_discovery_stub()
        result = self.run_shell('roslaunch', 'ugv_bringup', 'slicer_field_navigation.launch')
        self.assertEqual(result.returncode, 0, result.stderr)
        args = self.docker_args()
        devices = [args[i+1] for i, value in enumerate(args) if value == '--device']
        self.assertEqual(devices, ['/dev/null', '/dev/zero'])
        self.assertEqual(args.count('--group-add'), 2)
        self.assertNotIn('--privileged', args)
        self.assertNotIn('/dev/bus/usb', args)
        if Path('/run/udev').is_dir():
            self.assertIn('type=bind,src=/run/udev,dst=/run/udev,readonly', args)

    def test_failed_camera_discovery_never_runs_docker(self):
        self.camera_discovery_stub(devices=(), exit_code=2)
        result = self.run_shell()
        self.assertNotEqual(result.returncode, 0)
        self.assertFalse(self.argv_file.exists())

    def test_camera_discovery_cannot_map_a_regular_file(self):
        regular = self.root / 'not-camera'
        regular.write_text('not a device')
        self.camera_discovery_stub(devices=(str(regular),))
        result = self.run_shell()
        self.assertNotEqual(result.returncode, 0)
        self.assertFalse(self.argv_file.exists())


if __name__ == '__main__':
    unittest.main()
