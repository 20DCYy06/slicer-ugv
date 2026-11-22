"""Exercise the workflow controls without ROS or a local X11 session."""
from concurrent.futures import Future
import os
from pathlib import Path
import sys
import unittest
from unittest.mock import Mock, patch

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
from PyQt5 import QtCore, QtTest, QtWidgets

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from workflow_panel import WorkflowPanel


class WorkflowPanelTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def setUp(self):
        self.controller = Mock()
        self.controller.states = {
            key: ('red', '', True)
            for key in ('sensors', 'driver', 'odom', 'calibrate', 'map', 'vision', 'air')
        }
        self.controller.messages = []
        self.controller.mux = {}
        self.controller.owned = {}
        self.controller.calibrating = False
        self.panel = WorkflowPanel(self.controller)
        self.panel.resize(260, self.panel.sizeHint().height())
        self.panel.show()
        self.app.processEvents()
        self.addCleanup(self.panel.close)

    def click(self, button):
        QtTest.QTest.mouseClick(button, QtCore.Qt.LeftButton)

    def test_step_button_clicks_refresh_and_show_external_ownership(self):
        for key in ('sensors', 'driver', 'odom', 'calibrate', 'map', 'vision'):
            self.click(self.panel.buttons[key])
            self.controller.click.assert_called_with(key, mode='v1', confirmed=False)
        item = self.panel.buttons['sensors']
        self.assertEqual(item.property('stage'), 'red')
        self.controller.states['sensors'] = ('yellow', '启动中', False)
        self.panel.refresh()
        self.assertEqual(item.property('stage'), 'yellow')
        self.assertFalse(item.isEnabled())
        self.controller.states['sensors'] = ('green', '外部运行中', True)
        self.panel.refresh()
        self.assertEqual(item.property('stage'), 'green')
        self.assertIn('外部运行中', item.text())

    def test_air_starts_with_one_click_and_v2_mode_is_forwarded(self):
        air = self.panel.buttons['air']
        self.assertTrue(air.isEnabled())
        self.assertEqual(self.panel.findChildren(QtWidgets.QCheckBox), [])
        self.click(air)
        self.controller.click.assert_called_once_with('air', mode='v1', confirmed=False)
        self.controller.click.reset_mock()
        self.panel.air_mode.setCurrentIndex(1)
        self.click(air)
        self.controller.click.assert_called_once_with('air', mode='v2', confirmed=False)
        self.controller.states['air'] = ('green', '外部运行中', True)
        self.panel.refresh()
        self.assertTrue(air.isEnabled())
        self.assertFalse(self.panel.air_mode.isEnabled())
        self.assertEqual(air.property('stage'), 'green')

    def test_motion_confirmation_and_stop_all(self):
        self.controller.click.return_value = 'confirm'
        confirmation_states = []
        self.panel.calibration_active.connect(confirmation_states.append)
        self.click(self.panel.buttons['calibrate'])
        confirmation = self.panel.confirmation
        self.assertIsNotNone(confirmation)
        self.assertEqual(confirmation.windowModality(), QtCore.Qt.NonModal)
        self.click(confirmation.button(QtWidgets.QMessageBox.No))
        self.assertEqual(self.controller.click.call_count, 1)
        self.app.processEvents()
        self.click(self.panel.buttons['calibrate'])
        confirmation = self.panel.confirmation
        self.click(confirmation.button(QtWidgets.QMessageBox.Yes))
        self.controller.click.assert_called_with('calibrate', mode='v1', confirmed=True)
        self.app.processEvents()
        self.click(self.panel.buttons['calibrate'])
        self.assertIsNotNone(self.panel.confirmation)
        self.click(self.panel.stop_button)
        self.controller.stop_all.assert_called_once_with()
        self.assertIsNone(self.panel.confirmation)
        self.assertEqual(confirmation_states, [True, False, True, False, True, False])

    def test_slow_process_scan_leaves_stop_and_parking_responsive(self):
        future = Future()
        future.set_running_or_notify_cancel()
        self.panel.submit_process_scan = Mock(return_value=future)
        self.click(self.panel.buttons['driver'])
        self.panel.submit_process_scan.assert_called_once_with()
        self.controller.click.assert_not_called()
        self.assertIn('检查中', self.panel.buttons['driver'].toolTip())
        self.click(self.panel.stop_button)
        self.controller.stop_all.assert_called_once_with()
        self.click(self.panel.parking)
        self.controller.control.assert_called_once_with('release')
        future.set_result({})
        self.app.processEvents()
        self.controller.click.assert_not_called()
        self.assertTrue(self.panel.buttons['driver'].isEnabled())

    def test_click_uses_fresh_scan_and_recovers_from_scan_failure(self):
        failed, recovered = Future(), Future()
        failed.set_exception(OSError('proc scan not available'))
        groups = {'driver': {820}}
        recovered.set_result(groups)
        self.panel.submit_process_scan = Mock(side_effect=[failed, recovered])
        self.click(self.panel.buttons['driver'])
        QtTest.QTest.qWait(80)
        self.controller.click.assert_not_called()
        self.assertTrue(self.panel.buttons['driver'].isEnabled())
        self.click(self.panel.buttons['driver'])
        QtTest.QTest.qWait(80)
        self.controller.click.assert_called_once_with(
            'driver', mode='v1', confirmed=False, process_groups=groups)

    def test_driver_stopping_requires_a_fresh_scan_and_cannot_restart_after_exit(self):
        original = object()
        self.controller.owned['driver'] = original
        future = Future()
        self.panel.submit_process_scan = Mock(return_value=future)
        self.click(self.panel.buttons['driver'])
        self.controller.click.assert_not_called()
        self.controller.owned.clear()
        future.set_result({})
        QtTest.QTest.qWait(80)
        self.controller.click.assert_not_called()
        self.assertTrue(self.panel.buttons['driver'].isEnabled())

    def test_scan_timeout_never_launches_when_a_late_result_arrives(self):
        late = Future()
        late.set_running_or_notify_cancel()
        recovered = Future()
        recovered.set_result({})
        self.panel.submit_process_scan = Mock(side_effect=[late, recovered])
        self.click(self.panel.buttons['driver'])
        with patch('workflow_panel.time.monotonic', return_value=self.panel.pending_step[-1] + 1.):
            self.panel.finish_scan()
        self.assertIn('超时', self.controller.report.call_args[0][0])
        late.set_result({})
        QtTest.QTest.qWait(80)
        self.controller.click.assert_not_called()
        self.click(self.panel.buttons['driver'])
        QtTest.QTest.qWait(80)
        self.controller.click.assert_called_once_with(
            'driver', mode='v1', confirmed=False, process_groups={})

    def test_log_keeps_recent_three_lines_in_compact_panel(self):
        self.controller.messages = ['更早', '启动定位', '底盘就绪', '开始配准']
        self.panel.refresh()
        self.assertEqual(self.panel.log.toPlainText(), '启动定位\n底盘就绪\n开始配准')
        self.assertLessEqual(self.panel.minimumSizeHint().width(), 260)
        self.assertLessEqual(self.panel.sizeHint().height(), 310)

    def test_parking_control_tracks_mux_and_passes_explicit_action(self):
        self.assertTrue(self.panel.parking.isEnabled())
        self.click(self.panel.parking)
        self.controller.control.assert_called_with('release')
        self.controller.mux = {'manual_hold': True}
        self.panel.refresh()
        self.click(self.panel.parking)
        self.controller.control.assert_called_with('release')
        self.controller.mux = {'manual_hold': False}
        self.panel.refresh()
        self.click(self.panel.parking)
        self.controller.control.assert_called_with('hold')


if __name__ == '__main__':
    unittest.main()
