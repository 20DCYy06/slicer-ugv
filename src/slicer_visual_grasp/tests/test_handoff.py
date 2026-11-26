"""Exercise the original popup handoff without opening camera or serial devices."""
from pathlib import Path
import tempfile
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from slicer_visual_grasp.app import App
from slicer_visual_grasp.pipeline import Runtime


class PopupHandoffTests(unittest.TestCase):
    def popup(self, **state):
        app = App.__new__(App)
        app.handoff = True
        app.closed = app.busy = app.closing = app.stopping_pump = app.reconnecting = False
        app.signal_exit_requested = app.suspending = False
        app.refresh_job = None
        app.runtime = SimpleNamespace(**state)
        app.camera = Mock()
        app.camera.thread.is_alive.return_value = True
        app.win, app.log, app.stop = Mock(), Mock(), Mock()
        app.refresh_view = Mock()
        return app

    def test_held_popup_replies_only_after_cleanup_without_switching_pump_off(self):
        app = self.popup(needs_release=True, transport_ready=True)
        app.suspend()
        app.refresh()
        app.log.assert_not_called()
        app.camera.close.assert_called_once_with(wait=False)
        app.camera.thread.is_alive.return_value = False
        app.refresh()
        app.log.assert_called_once_with('GRASP_SUSPENDED')
        app.win.destroy.assert_called_once()

    def test_busy_or_uncertain_arm_cannot_suspend(self):
        app = self.popup(needs_release=True, transport_ready=True)
        app.busy = True
        app.suspend()
        self.assertFalse(app.closing)
        app.busy = False
        app.runtime.transport_ready = False
        app.suspend()
        self.assertFalse(app.closing)
        app.camera.close.assert_not_called()

    def test_reopen_keeps_held_state_and_failed_release_blocks_transport(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            (root / 'data').mkdir()
            def reopen():
                runtime = Runtime({}, root, None, threading.Event(), Mock())
                runtime.load_recovery()
                return runtime
            runtime = reopen()
            runtime.holding = runtime.transport_ready = True
            runtime.lowered_s = 1.
            runtime.checkpoint()
            runtime = reopen()
            self.assertTrue(runtime.transport_ready)
            self.assertEqual(runtime.lowered_s, 1.)
            with patch('slicer_visual_grasp.pipeline.device_config', side_effect=OSError):
                with self.assertRaises(OSError):
                    runtime.pump_off()
            runtime = reopen()
            self.assertTrue(runtime.needs_release)
            self.assertFalse(runtime.transport_ready)

    def test_signal_close_has_no_suspend_ack(self):
        app = self.popup(needs_release=False, transport_ready=False)
        app.signal_exit_requested = True
        app.camera.thread.is_alive.return_value = False
        app.refresh()
        app.win.destroy.assert_called_once()
        app.log.assert_not_called()


if __name__ == '__main__':
    unittest.main()
