"""Exercise the desktop command pipe with mocked camera and actuators."""
from contextlib import contextmanager
import json
import os
from pathlib import Path
import select
import signal
import subprocess
import sys
import tempfile
import time
import unittest

from slicer_visual_grasp.paths import DEFAULT_CONFIG


PROBE = '''
import os
from slicer_visual_grasp import app, pipeline
from airgrip import keyboard

class Camera:
    def __init__(self, *args): pass
    def start(self): print('CAMERA_STARTED', flush=True)
    def snapshot(self): return object(), None, None, None
    def close(self):
        if os.environ.get('VISUAL_GRASP_PROBE_FAIL_CLOSE'):
            raise OSError('mock camera close failure')
        print('CAMERA_CLOSED', flush=True)

class Servo:
    def __init__(self, *args): pass
    def check(self, *, required_pin): assert required_pin == 14
    def pulse(self, value): print('SERVO', value, flush=True)
    def receive(self): pass
    def close(self): pass

class AirGrip:
    def __init__(self, cfg): self.ser = self
    def set_pump(self, on):
        assert not on
        print('PUMP_OFF_ACK', flush=True)
    def close(self): pass

class Runtime(app.Runtime):
    def pick(self):
        assert not self.needs_release
        if os.environ.get('VISUAL_GRASP_PROBE_SLOW_PICK'):
            self.move_servo('down', 3., {'down': 900, 'refresh_hz': 20})
        self.holding = True
        self.checkpoint()

def no_window():
    raise RuntimeError('headless must not create a Tk window')

app.Camera, app.Runtime, app.tk.Tk = Camera, Runtime, no_window
pipeline.Servo = Servo
keyboard.AirGrip = AirGrip
app.main()
'''


class HeadlessTests(unittest.TestCase):
    @contextmanager
    def session(self, *args, cfg=None, slow_pick=False, fail_close=False):
        with tempfile.TemporaryDirectory() as folder:
            if cfg is None:
                cfg = Path(folder) / 'config.json'
                cfg.write_bytes(DEFAULT_CONFIG.read_bytes())
            env = dict(os.environ)
            if slow_pick:
                env['VISUAL_GRASP_PROBE_SLOW_PICK'] = '1'
            if fail_close:
                env['VISUAL_GRASP_PROBE_FAIL_CLOSE'] = '1'
            with subprocess.Popen(
                [sys.executable, '-s', '-c', PROBE, '--config', str(cfg),
                 '--headless', *args], stdin=subprocess.PIPE,
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, env=env,
            ) as process:
                buffer = bytearray()

                def read_until(expected):
                    deadline = time.monotonic() + 5.
                    while expected not in buffer:
                        remaining = deadline - time.monotonic()
                        if remaining <= 0 or not select.select([process.stdout], [], [], remaining)[0]:
                            self.fail(f'timed out waiting for {expected!r}: {buffer!r}')
                        chunk = os.read(process.stdout.fileno(), 4096)
                        if not chunk:
                            self.fail(f'worker exited while waiting for {expected!r}: {buffer!r}')
                        buffer.extend(chunk)

                try:
                    yield process, buffer, read_until
                finally:
                    if process.poll() is None:
                        process.kill()

    def test_start_pick_then_release_through_the_same_process(self):
        with self.session('--run', '--pick-on-start') as (process, buffer, read_until):
            read_until(b'GRASP_STATE held\n')
            self.assertEqual(buffer.count(b'GRASP_STATE busy\n'), 1)
            self.assertNotIn(b'GRASP_STATE idle\n', buffer)
            process.stdin.write(b'release\n')
            process.stdin.flush()
            read_until(b'GRASP_STATE idle\n')
            self.assertEqual(buffer.count(b'GRASP_STATE busy\n'), 2)
            process.send_signal(signal.SIGINT)
            read_until(b'CAMERA_CLOSED\n')
            self.assertEqual(process.wait(timeout=3.), 0)

    def test_recovery_session_can_release_without_starting_camera_or_pick(self):
        with self.session() as (process, buffer, read_until):
            read_until(b'GRASP_STATE idle\n')
            buffer.clear()
            process.stdin.write(b'release\n')
            process.stdin.flush()
            read_until(b'GRASP_STATE idle\n')
            self.assertNotIn(b'CAMERA_STARTED\n', buffer)
            self.assertNotIn(b'GRASP_STATE held\n', buffer)
            self.assertEqual(buffer.count(b'GRASP_STATE busy\n'), 1)

    def test_fatal_exit_while_holding_reports_recovery(self):
        with self.session('--pick-on-start', fail_close=True) as (process, buffer, read_until):
            read_until(b'GRASP_STATE held\n')
            process.send_signal(signal.SIGINT)
            read_until(b'GRASP_STATE recovery\n')
            self.assertEqual(process.wait(timeout=3.), 1)
            self.assertNotIn(b'GRASP_STATE error\n', buffer)

    def test_interrupted_pick_restarts_in_recovery_and_lifts_recorded_travel(self):
        for interruption in (signal.SIGINT, None):
            with self.subTest(interruption=interruption), tempfile.TemporaryDirectory() as folder:
                cfg = Path(folder) / 'config.json'
                cfg.write_bytes(DEFAULT_CONFIG.read_bytes())
                with self.session('--pick-on-start', cfg=cfg, slow_pick=True) as (process, _, read_until):
                    read_until(b'SERVO 900\n')
                    if interruption is None:
                        process.stdin.close()
                    else:
                        process.send_signal(interruption)
                    read_until(b'GRASP_STATE recovery\n')
                    self.assertEqual(process.wait(timeout=3.), 0)

                state = json.loads((cfg.parent / 'data/grasp_recovery.json').read_text())
                self.assertGreater(state['lowered_s'], 0.)
                self.assertFalse(state['travel_unknown'])
                with self.session('--pick-on-start', cfg=cfg) as (process, buffer, read_until):
                    read_until(b'GRASP_STATE recovery\n')
                    self.assertNotIn(b'CAMERA_STARTED\n', buffer)
                    process.stdin.write(b'release\n')
                    process.stdin.flush()
                    read_until(b'GRASP_STATE idle\n')
                    self.assertLess(buffer.index(b'PUMP_OFF_ACK\n'), buffer.index(b'SERVO 2100\n'))
                    self.assertIn(f"[STATE] LIFT：按记录回升 {state['lowered_s']:.3f} s".encode(), buffer)

                with self.session(cfg=cfg) as (_, buffer, read_until):
                    read_until(b'GRASP_STATE idle\n')
                    self.assertNotIn(b'GRASP_STATE recovery\n', buffer)

    def test_interrupted_without_cleanup_does_not_guess_lift_travel(self):
        with tempfile.TemporaryDirectory() as folder:
            cfg = Path(folder) / 'config.json'
            cfg.write_bytes(DEFAULT_CONFIG.read_bytes())
            with self.session('--pick-on-start', cfg=cfg, slow_pick=True) as (process, _, read_until):
                read_until(b'SERVO 900\n')
                process.kill()
                self.assertEqual(process.wait(timeout=3.), -signal.SIGKILL)

            with self.session('--pick-on-start', cfg=cfg) as (process, buffer, read_until):
                read_until(b'GRASP_STATE recovery\n')
                buffer.clear()
                process.stdin.write(b'release\n')
                process.stdin.flush()
                read_until(b'GRASP_STATE recovery\n')
                self.assertIn(b'PUMP_OFF_ACK\n', buffer)
                self.assertNotIn(b'SERVO 2100\n', buffer)
                self.assertNotIn(b'GRASP_STATE idle\n', buffer)
