"""Exercise the workflow controls without ROS or a local X11 session."""
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
        with patch.object(QtWidgets.QMessageBox, 'question', return_value=QtWidgets.QMessageBox.No):
            self.click(self.panel.buttons['calibrate'])
        self.assertEqual(self.controller.click.call_count, 1)
        with patch.object(QtWidgets.QMessageBox, 'question', return_value=QtWidgets.QMessageBox.Yes):
            self.click(self.panel.buttons['calibrate'])
        self.controller.click.assert_called_with('calibrate', mode='v1', confirmed=True)
        self.click(self.panel.stop_button)
        self.controller.stop_all.assert_called_once_with()

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
