"""The workspace pin wins over a legacy source tree, even off the ROS overlay."""
import os
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch


sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from hardware_path import hardware_config, hardware_root


class HardwarePathTest(unittest.TestCase):
    @staticmethod
    def make_hardware(directory):
        (directory / 'Host').mkdir(parents=True)
        (directory / 'config.yaml').touch()
        return directory

    def test_selects_matching_catkin_workspace_dependency(self):
        with tempfile.TemporaryDirectory() as temporary:
            workspace = Path(temporary)
            repo = workspace / 'src/slicer-ugv'
            self.make_hardware(repo / 'Hardware')
            pinned = self.make_hardware(workspace / 'deps/slicer_ugv_hardware')
            with patch.dict(os.environ, {'SLICER_HARDWARE_ROOT': ''}):
                self.assertEqual(hardware_root(repo), pinned)
                self.assertEqual(hardware_root(workspace), pinned)

    def test_explicit_missing_root_fails_instead_of_changing_hardware(self):
        with tempfile.TemporaryDirectory() as temporary:
            repo = Path(temporary)
            legacy = self.make_hardware(repo / 'Hardware')
            with patch.dict(os.environ, {'SLICER_HARDWARE_ROOT': ''}):
                self.assertEqual(hardware_root(repo), legacy)
            with patch.dict(os.environ, {'SLICER_HARDWARE_ROOT': str(repo / 'wrong')}):
                with self.assertRaisesRegex(FileNotFoundError, 'wrong'):
                    hardware_root(repo)
                self.assertEqual(hardware_root(repo, legacy), legacy)

    def test_selects_field_config_independently_of_source_tree(self):
        with tempfile.TemporaryDirectory() as temporary:
            workspace = Path(temporary)
            root = self.make_hardware(workspace / 'deps/slicer_ugv_hardware')
            selected = workspace / 'site/c30d.yaml'
            selected.parent.mkdir()
            selected.touch()
            default = root / 'config.yaml'
            with patch.dict(os.environ, {'SLICER_HARDWARE_CONFIG': ''}):
                self.assertEqual(hardware_config(root), default)
            with patch.dict(os.environ, {'SLICER_HARDWARE_CONFIG': str(selected)}):
                self.assertEqual(hardware_config(root), selected)
                self.assertEqual(hardware_config(root, default), default)
                with self.assertRaises(FileNotFoundError):
                    hardware_config(root, selected.with_name('missing.yaml'))


if __name__ == '__main__':
    unittest.main()
