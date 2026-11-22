"""Release metadata checks with local repositories, no device access."""
import hashlib
import json
from pathlib import Path
import subprocess
import tempfile
import unittest

import yaml


RECORDER = Path(__file__).resolve().parents[1] / 'record_release.py'


class ReleaseRecordTest(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.workspace = Path(temporary.name) / 'release-fixture'
        self.vehicle = self.workspace / 'src/slicer-ugv'
        self.hardware = self.workspace / 'deps/slicer_ugv_hardware'
        self.vehicle.mkdir(parents=True)
        self.hardware.mkdir(parents=True)
        self.firmware = self.hardware / 'Firmware/v-test/bin/firmware.bin'
        self.firmware.parent.mkdir(parents=True)
        self.firmware.write_bytes(b'only a fixture')
        self.firmware_sha = hashlib.sha256(self.firmware.read_bytes()).hexdigest()
        (self.hardware / 'last_install.json').write_text(json.dumps({
            'version': 'v-test', 'image_sha256': self.firmware_sha,
            'verified': True, 'readback_sha256': 'a' * 64, 'device': 'TEST-HIDDEN',
            'usb_serial': 'TEST-SERIAL-HIDDEN', 'directory': '/private/path',
        }))
        self.hardware_url = 'https://example.invalid/hardware.git'
        self.hardware_sha = self.add_repository(self.hardware, self.hardware_url)
        self.pins = self.vehicle / 'dependencies.repos'
        self.pins.write_text(yaml.safe_dump({'repositories': {
            'deps/slicer_ugv_hardware': {
                'type': 'git', 'url': self.hardware_url, 'version': self.hardware_sha,
            },
        }}))
        self.vehicle_sha = self.add_repository(self.vehicle, 'https://example.invalid/vehicle.git')
        for path in ('install', 'runtime/no_rtk/config', 'runtime/rtk_config', 'sdk/install'):
            (self.workspace / path).mkdir(parents=True)
        (self.workspace / 'install/setup.bash').touch()
        (self.workspace / 'runtime/no_rtk/config/control.yaml').write_text('control: fixture\n')
        (self.workspace / 'runtime/rtk_config/driver.yaml').write_text('driver: fixture\n')

    def add_repository(self, path, origin):
        subprocess.run(['git', 'init', '-q', str(path)], check=True)
        subprocess.run(['git', '-C', str(path), 'remote', 'add', 'origin', origin], check=True)
        subprocess.run(['git', '-C', str(path), 'add', '.'], check=True)
        subprocess.run(['git', '-C', str(path), '-c', 'user.name=Fixture',
                        '-c', 'user.email=fixture@example.invalid', 'commit',
                        '-q', '-m', 'fixture'], check=True)
        return subprocess.check_output(['git', '-C', str(path), 'rev-parse', 'HEAD'],
                                       text=True).strip()

    def record(self, *extra):
        return subprocess.run(['/usr/bin/python3', str(RECORDER), str(self.workspace),
                               '--build-command', 'bash tools/build_native.sh W -j2',
                               '--verification', 'build=passed',
                               '--verification', 'hardware=not-run', *extra],
                              capture_output=True, text=True)

    def test_record_contains_pins_runtime_hashes_and_only_safe_firmware_details(self):
        result = self.record('--firmware-record', str(self.hardware / 'last_install.json'))
        self.assertEqual(result.returncode, 0, result.stderr)
        raw = (self.workspace / 'release.json').read_text()
        record = json.loads(raw)
        self.assertEqual(record['release_id'], self.workspace.name)
        self.assertEqual(record['workspace'], str(self.workspace))
        self.assertEqual(record['vehicle']['sha'], self.vehicle_sha)
        self.assertEqual(record['dependencies']['deps/slicer_ugv_hardware'],
                         {'url': self.hardware_url, 'sha': self.hardware_sha})
        self.assertEqual(record['firmware']['image_sha256'], self.firmware_sha)
        self.assertEqual(record['firmware']['historical_readback']['sha256'], 'a' * 64)
        self.assertNotIn('TEST-HIDDEN', raw)
        self.assertNotIn('TEST-SERIAL-HIDDEN', raw)
        self.assertNotIn('/private/path', raw)
        self.assertIn('runtime/no_rtk/config/control.yaml', record['runtime_config_sha256'])
        self.assertEqual(record['sdk']['prefix'], str(self.workspace / 'sdk/install'))
        self.assertEqual(record['verification'], {'build': 'passed', 'hardware': 'not-run'})
        self.assertGreater((self.workspace / 'apt-packages.tsv').stat().st_size, 0)
        self.assertNotEqual(self.record().returncode, 0)
        self.assertEqual((self.workspace / 'release.json').read_text(), raw)

    def test_no_trusted_record_means_no_readback_claim(self):
        result = self.record()
        self.assertEqual(result.returncode, 0, result.stderr)
        record = json.loads((self.workspace / 'release.json').read_text())
        self.assertNotIn('historical_readback', record['firmware'])
        self.assertEqual(record['firmware']['image_sha256'], self.firmware_sha)

    def test_refuses_pin_changes_after_checkout(self):
        self.pins.write_text(self.pins.read_text().replace(self.hardware_sha, '0' * 40))
        result = self.record()
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('dependencies.repos differs', result.stderr)
        self.assertFalse((self.workspace / 'release.json').exists())

    def test_refuses_dirty_published_dependency_even_if_head_matches(self):
        (self.hardware / 'last_install.json').write_text('{"version":"v-other"}')
        result = self.record()
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('Tracked source differs', result.stderr)
        self.assertFalse((self.workspace / 'release.json').exists())


if __name__ == '__main__':
    unittest.main()
