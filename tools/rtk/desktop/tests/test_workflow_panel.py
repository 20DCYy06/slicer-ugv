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
            for key in ('rtk', 'base', 'navigation', 'vision', 'air')
        }
        self.controller.messages = []
        self.controller.mux = {}
        self.controller.owned = {}
        self.controller.calibrating = False
        self.controller.startup_calibration_pending = False
        self.controller.stopping_all = False
        self.controller.switching = False
        self.controller.can_send_wgs84.return_value = True
        self.panel = WorkflowPanel(self.controller)
        self.panel.resize(260, self.panel.sizeHint().height())
        self.panel.show()
        self.app.processEvents()
        self.addCleanup(self.panel.close)

    def click(self, button):
        QtTest.QTest.mouseClick(button, QtCore.Qt.LeftButton)

    def test_step_button_clicks_refresh_and_show_external_ownership(self):
        for key in ('rtk', 'base', 'navigation'):
            self.click(self.panel.buttons[key])
            self.controller.click.assert_called_with(key, mode='v1', confirmed=False)
        item = self.panel.buttons['rtk']
        self.assertEqual(item.property('stage'), 'red')
        self.controller.states['rtk'] = ('yellow', '启动中', False)
        self.panel.refresh()
        self.assertEqual(item.property('stage'), 'yellow')
        self.assertFalse(item.isEnabled())
        self.controller.states['rtk'] = ('green', '外部运行中', True)
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

    def test_stop_remains_available_during_roundtrip(self):
        confirmation_states = []
        self.panel.calibration_active.connect(confirmation_states.append)
        self.controller.calibrating = True
        self.panel.refresh()
        self.click(self.panel.stop_button)
        self.controller.stop_all.assert_called_once_with()
        self.controller.calibrating = False
        self.panel.refresh()
        self.assertEqual(confirmation_states, [True, False])

    def test_waypoint_confirmation_sends_only_latest_accepted_selection(self):
        first = Mock()
        second = Mock()
        self.panel.confirm_waypoint('31.1111111, 121.1111111', first)
        dialog = self.panel.waypoint_dialog
        self.assertIn('31.1111111, 121.1111111', dialog.text())
        self.click(dialog.button(QtWidgets.QMessageBox.No))
        first.assert_not_called()

        self.panel.confirm_waypoint('31.1111111, 121.1111111', first)
        self.panel.confirm_waypoint('31.2222222, 121.2222222', second)
        dialog = self.panel.waypoint_dialog
        self.assertIn('31.2222222, 121.2222222', dialog.text())
        self.click(dialog.button(QtWidgets.QMessageBox.Yes))
        first.assert_not_called()
        second.assert_called_once_with()

        self.panel.confirm_waypoint('', first)
        self.click(self.panel.stop_button)
        self.assertIsNone(self.panel.waypoint_dialog)
        first.assert_not_called()

    def test_waypoint_cannot_submit_without_coordinates(self):
        self.panel.confirm_waypoint('', Mock())
        self.assertIsNone(self.panel.waypoint_dialog)

    def test_slow_process_scan_leaves_stop_responsive(self):
        future = Future()
        future.set_running_or_notify_cancel()
        self.panel.submit_process_scan = Mock(return_value=future)
        self.click(self.panel.buttons['base'])
        self.panel.submit_process_scan.assert_called_once_with()
        self.controller.click.assert_not_called()
        self.assertIn('检查中', self.panel.buttons['base'].toolTip())
        self.click(self.panel.stop_button)
        self.controller.stop_all.assert_called_once_with()
        future.set_result({})
        self.app.processEvents()
        self.controller.click.assert_not_called()
        self.assertTrue(self.panel.buttons['base'].isEnabled())

    def test_click_uses_fresh_scan_and_recovers_from_scan_failure(self):
        failed, recovered = Future(), Future()
        failed.set_exception(OSError('proc scan not available'))
        groups = {'driver': {820}}
        recovered.set_result(groups)
        self.panel.submit_process_scan = Mock(side_effect=[failed, recovered])
        self.click(self.panel.buttons['base'])
        QtTest.QTest.qWait(80)
        self.controller.click.assert_not_called()
        self.assertTrue(self.panel.buttons['base'].isEnabled())
        self.click(self.panel.buttons['base'])
        QtTest.QTest.qWait(80)
        self.controller.click.assert_called_once_with(
            'base', mode='v1', confirmed=False, process_groups=groups)

    def test_driver_stopping_requires_a_fresh_scan_and_cannot_restart_after_exit(self):
        original = object()
        self.controller.owned['driver'] = original
        future = Future()
        self.panel.submit_process_scan = Mock(return_value=future)
        self.click(self.panel.buttons['base'])
        self.controller.click.assert_not_called()
        self.controller.owned.clear()
        future.set_result({})
        QtTest.QTest.qWait(80)
        self.controller.click.assert_not_called()
        self.assertTrue(self.panel.buttons['base'].isEnabled())

    def test_scan_timeout_never_launches_when_a_late_result_arrives(self):
        late = Future()
        late.set_running_or_notify_cancel()
        recovered = Future()
        recovered.set_result({})
        self.panel.submit_process_scan = Mock(side_effect=[late, recovered])
        self.click(self.panel.buttons['base'])
        with patch('workflow_panel.time.monotonic', return_value=self.panel.pending_step[-1] + 1.):
            self.panel.finish_scan()
        self.assertIn('超时', self.controller.report.call_args[0][0])
        late.set_result({})
        QtTest.QTest.qWait(80)
        self.controller.click.assert_not_called()
        self.click(self.panel.buttons['base'])
        QtTest.QTest.qWait(80)
        self.controller.click.assert_called_once_with(
            'base', mode='v1', confirmed=False, process_groups={})

    def test_log_keeps_recent_three_lines_in_compact_panel(self):
        self.controller.messages = ['更早', '启动定位', '底盘就绪', '开始配准']
        self.panel.refresh()
        self.assertEqual(self.panel.log.toPlainText(), '启动定位\n底盘就绪\n开始配准')
        self.assertLessEqual(self.panel.minimumSizeHint().width(), 260)
        self.assertLessEqual(self.panel.sizeHint().height(), 310)

    def test_only_four_steps_and_stop_are_shown(self):
        self.assertEqual(list(self.panel.buttons), ['rtk', 'base', 'navigation', 'vision', 'air'])
        self.assertEqual({button.objectName() for button in self.panel.findChildren(QtWidgets.QPushButton)},
                         {'workflow_rtk', 'workflow_base', 'workflow_navigation', 'workflow_vision', 'workflow_air', 'stop_all'})


if __name__ == '__main__':
    unittest.main()
