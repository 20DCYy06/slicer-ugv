"""Launch and stop one owned process group with bounded shutdown."""
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time


class OwnedProcess:
    def __init__(self, key, argv, env, *, log_prefix='rtk', stdin=False):
        log_dir = Path(env.get('ROS_HOME') or Path.home() / '.ros') / 'log'
        log_dir.mkdir(parents=True, exist_ok=True)
        with tempfile.NamedTemporaryFile(prefix=f'{log_prefix}-{key}-', suffix='.log',
                                         dir=str(log_dir), delete=False) as stream:
            self.log = Path(stream.name)
            self.process = subprocess.Popen(argv, env=env,
                                            stdin=subprocess.PIPE if stdin else subprocess.DEVNULL,
                                            stdout=stream, stderr=subprocess.STDOUT,
                                            start_new_session=True)
        self.offset = 0
        self.pending_line = b''
        self.stop_at = None
        self.signal_stage = 0

    def send(self, command):
        self.process.stdin.write((command + '\n').encode('ascii'))
        self.process.stdin.flush()

    def _signal(self, sig):
        try:
            os.killpg(self.process.pid, sig)
        except ProcessLookupError:
            return False
        return True

    def stop(self):
        if self.stop_at is None:
            self.stop_at = time.monotonic()
            self._signal(signal.SIGINT)

    def poll(self):
        status = self.process.poll()
        if self.stop_at is None:
            return status
        # sg may exit on SIGTERM before its child finishes closing the serial port.
        if status is not None and (self.signal_stage == 2 or not self._signal(0)):
            return status
        elapsed = time.monotonic() - self.stop_at
        # Allow roslaunch and Tk to finish their own shutdown handlers.
        if elapsed > 32. and self.signal_stage == 1:
            self.signal_stage = 2
            self._signal(signal.SIGKILL)
        elif elapsed > 25. and self.signal_stage == 0:
            self.signal_stage = 1
            self._signal(signal.SIGTERM)
        return None

    def lines(self):
        try:
            with self.log.open('rb') as stream:
                # Skip old verbose output rather than stalling the GUI.
                stream.seek(max(self.offset, self.log.stat().st_size - 8192))
                data = self.pending_line + stream.read(8192)
                self.offset = stream.tell()
        except OSError:
            return []
        *lines, self.pending_line = data.split(b'\n')
        if self.process.poll() is not None and self.pending_line:
            lines.append(self.pending_line)
            self.pending_line = b''
        return [line.decode(errors='replace').strip() for line in lines if line.strip()]
