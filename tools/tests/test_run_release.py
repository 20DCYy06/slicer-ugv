"""Device-free checks for the stable release dispatcher."""
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest


DISPATCHER = Path(__file__).resolve().parents[1] / 'run_release.sh'


class ReleaseDispatcherTest(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory(prefix='slicer dispatch ')
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        self.run_path = self.root / 'run'
        shutil.copyfile(DISPATCHER, self.run_path)
        self.run_path.chmod(0o755)

    def release(self, name):
        release = self.root / 'releases' / name
        checkout = release / 'src/slicer-ugv'
        (checkout / 'tools').mkdir(parents=True)
        (release / 'runtime/no_rtk').mkdir(parents=True)
        (release / 'install').mkdir()
        subprocess.run(['git', 'init', '-q', str(checkout)], check=True)
        subprocess.run(['git', '-C', str(checkout), '-c', 'user.name=Fixture',
                        '-c', 'user.email=fixture@example.invalid', 'commit', '-q',
                        '--allow-empty', '-m', name], check=True)
        sha = subprocess.check_output(['git', '-C', str(checkout), 'rev-parse', 'HEAD'],
                                      text=True).strip()
        (release / 'release.json').write_text(json.dumps({
            'release_id': name, 'vehicle': {'sha': sha},
        }))
        (release / 'install/setup.bash').write_text('''
[[ -z "${CMAKE_PREFIX_PATH:-}${ROS_PACKAGE_PATH:-}${PYTHONPATH:-}${LD_LIBRARY_PATH:-}" ]]
[[ "$PATH" != *dev-overlay* ]]
export CMAKE_PREFIX_PATH="$SLICER_NATIVE_WS/install"
export TEST_SELECTED_OVERLAY="$SLICER_NATIVE_WS/install"
''')
        for script in ('rtk_desktop_ui.sh', 'no_rtk.sh'):
            # The stub only reports the environment selected by the dispatcher.
            (checkout / 'tools' / script).write_text('''
printf '%s\n' "$0" "$@" "$TEST_SELECTED_OVERLAY" "$SLICER_NO_RTK_RUNTIME" \
    "$WORKSPACE2_ROOT" "$SLICER_SITE_DIR" "$SLICER_STATE_DIR" \
    "$ROS_HOME" "$ROS_LOG_DIR" "$SLICER_RUNTIME_DIR"
''')
        return release, checkout, sha

    def select(self, target):
        next_link = self.root / 'next'
        next_link.symlink_to(target, target_is_directory=True)
        next_link.replace(self.root / 'current')

    def dispatch(self, *arguments):
        environment = dict(os.environ,
                           CMAKE_PREFIX_PATH='/dev-overlay/install',
                           ROS_PACKAGE_PATH='/dev-overlay/src',
                           PYTHONPATH='/dev-overlay/python',
                           LD_LIBRARY_PATH='/dev-overlay/lib',
                           PATH='/dev-overlay/bin:' + os.environ['PATH'],
                           SLICER_NATIVE_WS='/dev-overlay',
                           SLICER_NO_RTK_RUNTIME='/dev-overlay/runtime',
                           WORKSPACE2_ROOT='/dev-overlay/src',
                           SLICER_SITE_DIR='/dev-overlay/site',
                           SLICER_STATE_DIR='/dev-overlay/state',
                           SLICER_LOGS_DIR='/dev-overlay/logs',
                           SLICER_RUNTIME_DIR='/dev-overlay/data',
                           ROS_HOME='/dev-overlay/ros',
                           ROS_LOG_DIR='/dev-overlay/ros/log',
                           GIT_DIR='/dev-overlay/.git',
                           SLICER_GNSS_CONFIG='/shared/gnss.yaml',
                           BASH_ENV=str(self.root / 'dev-env.sh'))
        (self.root / 'dev-env.sh').write_text('exit 99\n')
        return subprocess.run([str(self.run_path), *arguments], env=environment,
                              capture_output=True, text=True)

    def test_current_selects_release_and_ignores_development_overlay(self):
        first, checkout, sha = self.release('release-a')
        second, other_checkout, _ = self.release('release-b')
        self.select(Path('releases/release-a'))
        status = self.dispatch('status')
        self.assertEqual(status.returncode, 0, status.stderr)
        self.assertIn(str(first), status.stdout)
        self.assertIn(sha, status.stdout)
        self.assertFalse((self.root / 'site').exists(), 'status must be read-only')

        rtk = self.dispatch('rtk')
        self.assertEqual(rtk.returncode, 0, rtk.stderr)
        self.assertEqual(rtk.stdout.splitlines(), [
            str(checkout / 'tools/rtk_desktop_ui.sh'),
            str(first / 'install'), str(first / 'runtime/no_rtk'), str(checkout),
            str(self.root / 'site'), str(self.root / 'site/state/release-a'),
            str(self.root / 'site/state/release-a/ros'),
            str(self.root / 'logs/release-a/ros'), str(self.root / 'data/release-a'),
        ])

        self.select(second)
        no_rtk = self.dispatch('no-rtk', 'nav', '--background', 'gui:=false')
        self.assertEqual(no_rtk.returncode, 0, no_rtk.stderr)
        self.assertEqual(no_rtk.stdout.splitlines()[:6], [
            str(other_checkout / 'tools/no_rtk.sh'), 'nav', '--background', 'gui:=false',
            str(second / 'install'), str(second / 'runtime/no_rtk'),
        ])
        shell = self.dispatch('shell', 'printenv', 'TEST_SELECTED_OVERLAY', 'SLICER_GNSS_CONFIG')
        self.assertEqual(shell.returncode, 0, shell.stderr)
        self.assertEqual(shell.stdout.splitlines(), [str(second / 'install'), '/shared/gnss.yaml'])

    def test_current_must_be_a_symlink_to_an_immediate_release_child(self):
        release, _, _ = self.release('release-a')
        (self.root / 'current').mkdir()
        self.assertNotEqual(self.dispatch('status').returncode, 0)
        (self.root / 'current').rmdir()
        for target in (release / 'runtime', self.root / 'missing', self.root):
            with self.subTest(target=target):
                self.select(target)
                result = self.dispatch('status')
                self.assertNotEqual(result.returncode, 0)
                self.assertEqual(result.stdout, '')

    def test_mismatched_vehicle_manifest_prevents_dispatch(self):
        release, _, _ = self.release('release-a')
        self.select(release)
        (release / 'release.json').write_text(json.dumps({
            'release_id': 'release-a', 'vehicle': {'sha': '0' * 40},
        }))
        result = self.dispatch('rtk')
        self.assertNotEqual(result.returncode, 0)
        self.assertRegex(result.stderr, 'commit|SHA')
        self.assertEqual(result.stdout, '')
        self.assertFalse((self.root / 'site').exists())

    def test_refuses_source_or_overlay_escaping_selected_release(self):
        release, _, _ = self.release('release-a')
        other, _, _ = self.release('release-b')
        self.select(release)
        (release / 'install').rename(release / 'old-install')
        (release / 'install').symlink_to(other / 'install', target_is_directory=True)
        result = self.dispatch('rtk')
        self.assertNotEqual(result.returncode, 0)
        self.assertEqual(result.stdout, '')

    def test_modified_source_is_not_a_valid_release(self):
        release, checkout, _ = self.release('release-a')
        self.select(release)
        subprocess.run(['git', '-C', str(checkout), 'add', 'tools/rtk_desktop_ui.sh'], check=True)
        result = self.dispatch('rtk')
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('modified tracked files', result.stderr)
        self.assertFalse((self.root / 'site').exists())

    def test_stale_installed_setup_cannot_activate_a_different_overlay(self):
        release, _, _ = self.release('release-a')
        self.select(release)
        (release / 'install/setup.bash').write_text('export CMAKE_PREFIX_PATH=/obsolete/install\n')
        result = self.dispatch('shell', 'printenv', 'CMAKE_PREFIX_PATH')
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('did not activate', result.stderr)
        self.assertEqual(result.stdout, '')

    def test_nortk_cannot_point_release_navigation_at_another_workspace(self):
        release, _, _ = self.release('release-a')
        self.select(release)
        for key in ('runtime_root', 'host_config', 'workspace2_root', 'waypoint_file'):
            with self.subTest(key=key):
                result = self.dispatch('no-rtk', 'nav', key + ':=/other-workspace/config')
                self.assertNotEqual(result.returncode, 0)
                self.assertIn('the release selects ' + key, result.stderr)
                self.assertFalse((self.root / 'site').exists())

    def test_build_does_not_run_in_the_selected_release(self):
        release, _, _ = self.release('release-a')
        self.select(release)
        result = self.dispatch('no-rtk', 'build')
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('development workspace', result.stderr)
        self.assertFalse((self.root / 'site').exists())


if __name__ == '__main__':
    unittest.main()
