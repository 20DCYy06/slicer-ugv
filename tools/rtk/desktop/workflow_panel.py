"""Compact controls for the RTK desktop workflow."""
import time

from PyQt5 import QtCore, QtGui, QtWidgets
from dashboard_model import StableStatus


STEPS = {
    'sensors': '1. 启动定位与传感器',
    'driver': '2. 启动底盘驱动',
    'vision': '3. 移动并抓取',
    'navigation': '4. 启动导航',
    'air': '5. 飞机通信节点',
    'wgs84': '发送 WGS84 目标',
}
DIAGNOSTICS = {
    'calibrate': '直行配准诊断 (2.5 m)',
    'map': '手动切换 map',
}
STATUS = {'red': '未启动', 'yellow': '进行中', 'green': '已就绪'}
STYLES = """
#WorkflowPanel QPushButton {
    color: #e9f3fc;
    background: #243a4b;
    border: 1px solid #3c5265;
    border-radius: 4px;
    padding: 0 6px;
    text-align: left;
}
#WorkflowPanel QPushButton[stage="red"] { background: #4f2632; border-color: #77424f; }
#WorkflowPanel QPushButton[stage="yellow"] { background: #56431c; border-color: #876923; }
#WorkflowPanel QPushButton[stage="green"] { background: #194737; border-color: #357d64; }
#WorkflowPanel QPushButton:hover, #WorkflowPanel QPushButton:focus { border-color: #a5d6eb; }
#WorkflowPanel QPushButton:disabled { color: #b2bfca; }
#WorkflowPanel QComboBox { color: #e9f3fc; background: #243a4b; }
#WorkflowPanel QComboBox QAbstractItemView { color: #e9f3fc; background: #243a4b; }
#WorkflowPanel QPushButton#stop_all { background: #61303b; border-color: #ad5f70; }
#WorkflowPanel QPushButton#stop_all:hover, #WorkflowPanel QPushButton#stop_all:focus { border-color: #ffd0d9; }
#WorkflowPanel QPlainTextEdit {
    color: #b2c7d8;
    background: #172534;
    border: 1px solid #2c4051;
    border-radius: 4px;
}
"""


class WorkflowPanel(QtWidgets.QWidget):
    calibration_active = QtCore.pyqtSignal(bool)

    def __init__(self, controller, cancel_pending_goal=None, submit_process_scan=None):
        super().__init__(objectName='WorkflowPanel')
        self.controller = controller
        self.cancel_pending_goal = cancel_pending_goal
        self.submit_process_scan = submit_process_scan
        self.scan = None
        self.pending_step = None
        self.confirmation = None
        self.target_dialog = None
        self.calibration_blocked = False
        self.scan_timer = QtCore.QTimer(self, interval=40)
        self.scan_timer.timeout.connect(self.finish_scan)
        self.stable = StableStatus()
        self.setStyleSheet(STYLES)
        self.setFont(QtGui.QFont('Noto Sans CJK SC', 10))
        self.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Maximum)
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(2)

        heading = QtWidgets.QLabel('流程控制')
        heading.setFont(QtGui.QFont(heading.font().family(), 10, QtGui.QFont.Bold))
        self.heading = heading
        layout.addWidget(heading)

        self.buttons = {}
        for key in STEPS:
            button = QtWidgets.QPushButton(STEPS[key], objectName='workflow_' + key)
            button.setFixedHeight(23)
            button.setSizePolicy(QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Fixed)
            button.clicked.connect(lambda _checked=False, item=key: self.click(item))
            self.buttons[key] = button
            if key not in ('air', 'wgs84'):
                layout.addWidget(button)

        options = QtWidgets.QHBoxLayout()
        options.setSpacing(4)
        self.air_mode = QtWidgets.QComboBox()
        self.air_mode.addItem('V1', 'v1')
        self.air_mode.addItem('V2', 'v2')
        self.air_mode.setFixedHeight(23)
        self.air_mode.setAccessibleName('空地协同协议')
        self.air_mode.setToolTip('选择飞机端使用的通信协议，再点击飞机通信节点')
        options.addWidget(self.buttons['air'], 1)
        options.addWidget(self.air_mode)
        layout.addLayout(options)

        actions = QtWidgets.QHBoxLayout()
        actions.setSpacing(4)
        actions.addWidget(self.buttons['wgs84'], 1)
        self.diagnostics = QtWidgets.QPushButton('导航诊断')
        self.diagnostics.setFixedHeight(23)
        menu = QtWidgets.QMenu(self.diagnostics)
        self.diagnostics_actions = {}
        for key, label in DIAGNOSTICS.items():
            action = menu.addAction(label)
            action.triggered.connect(lambda _checked=False, item=key: self.click(item))
            self.diagnostics_actions[key] = action
        self.diagnostics.setMenu(menu)
        actions.addWidget(self.diagnostics)
        layout.addLayout(actions)

        self.stop_button = QtWidgets.QPushButton('一键全部停止', objectName='stop_all')
        self.stop_button.setFixedHeight(23)
        self.stop_button.setToolTip('依次关闭本面板启动的进程')
        self.stop_button.clicked.connect(self.stop_all)
        layout.addWidget(self.stop_button)
        self.log = QtWidgets.QPlainTextEdit(readOnly=True)
        self.log.setAccessibleName('运行日志摘要')
        self.log.setFont(QtGui.QFont('Noto Sans CJK SC', 9))
        self.log.setFixedHeight(64)
        self.log.setSizePolicy(QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Fixed)
        self.log.setLineWrapMode(QtWidgets.QPlainTextEdit.NoWrap)
        self.log.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.log.setPlaceholderText('等待操作')
        layout.addWidget(self.log)
        self.refresh()

    def click(self, key):
        if self.cancel_pending_goal is not None:
            self.cancel_pending_goal()
        if key == 'wgs84':
            if key in self.controller.owned:
                self.controller.stop(key)
            else:
                self.open_wgs84()
            self.refresh()
            return
        if self.pending_step is not None and key == self.pending_step[0]:
            return
        if key == 'calibrate' and self.confirmation is not None:
            self.confirmation.raise_()
            self.confirmation.activateWindow()
            return
        if self.confirmation is not None or key in self.controller.owned:
            self.cancel_pending()
        self.request_step(key, self.air_mode.currentData(), confirmed=False)

    def open_wgs84(self):
        if self.target_dialog is not None:
            self.target_dialog.raise_()
            self.target_dialog.activateWindow()
            return
        dialog = QtWidgets.QDialog(self)
        dialog.setWindowTitle('发送 WGS84 导航目标')
        dialog.setAttribute(QtCore.Qt.WA_DeleteOnClose)
        form = QtWidgets.QFormLayout(dialog)
        inputs = []
        for label in ('纬度 (°)', '经度 (°)', '椭球高 (m)'):
            entry = QtWidgets.QLineEdit()
            entry.setAccessibleName(label)
            form.addRow(label, entry)
            inputs.append(entry)
        hint = QtWidgets.QLabel('提交后，车辆可能先执行自动配准准备运动。')
        form.addRow(hint)
        buttons = QtWidgets.QDialogButtonBox(QtWidgets.QDialogButtonBox.Ok | QtWidgets.QDialogButtonBox.Cancel)
        buttons.button(QtWidgets.QDialogButtonBox.Ok).setText('发送任务')
        buttons.button(QtWidgets.QDialogButtonBox.Cancel).setText('取消')
        form.addRow(buttons)

        def submit():
            if self.cancel_pending_goal is not None:
                self.cancel_pending_goal()
            if self.controller.send_wgs84(*(entry.text() for entry in inputs)):
                dialog.accept()
            self.refresh()

        buttons.accepted.connect(submit)
        buttons.rejected.connect(dialog.reject)
        dialog.finished.connect(lambda _result: self._finished_wgs84(dialog))
        self.target_dialog = dialog
        dialog.show()

    def _finished_wgs84(self, dialog):
        if self.target_dialog is dialog:
            self.target_dialog = None

    def request_step(self, key, mode, confirmed):
        owner = self.controller.owned.get(key)
        if self.submit_process_scan is None or (owner is not None and key not in ('sensors', 'driver')):
            self.run_step(key, mode, confirmed)
            return
        if self.pending_step is not None or self.scan is not None and not self.scan.done():
            self.controller.report('正在检查本机进程，请稍候。')
        else:
            self.scan = self.submit_process_scan()
            self.pending_step = key, mode, confirmed, owner, time.monotonic() + 2.
            self.scan_timer.start()
        self.refresh()

    def finish_scan(self):
        if self.pending_step is None:
            return
        key, mode, confirmed, owner, deadline = self.pending_step
        if time.monotonic() > deadline:
            self.cancel_pending()
            self.controller.report('本机进程检查超时，请重新点击。')
        elif not self.scan.done():
            return
        else:
            scan, self.scan = self.scan, None
            self.pending_step = None
            self.scan_timer.stop()
            try:
                groups = scan.result()
            except Exception as error:
                self.controller.report(f'无法检查本机进程，请重新点击。{error}')
            else:
                if self.controller.owned.get(key) is not owner:
                    self.controller.report('步骤状态已变化，请重新点击。')
                else:
                    self.run_step(key, mode, confirmed, process_groups=groups)
        self.refresh()

    def run_step(self, key, mode, confirmed, **kwargs):
        if confirmed and self.cancel_pending_goal is not None:
            self.cancel_pending_goal()
        result = self.controller.click(key, mode=mode, confirmed=confirmed, **kwargs)
        if result == 'confirm' and not confirmed:
            box = QtWidgets.QMessageBox(
                QtWidgets.QMessageBox.Question, '确认地理配准',
                '小车将在配准时直行约 2.5 m。确认前方空旷且可以急停后开始。',
                QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No, self)
            box.setWindowFlag(QtCore.Qt.WindowStaysOnTopHint)
            box.setWindowModality(QtCore.Qt.NonModal)
            box.setAttribute(QtCore.Qt.WA_DeleteOnClose)
            box.setDefaultButton(QtWidgets.QMessageBox.No)
            box.finished.connect(lambda answer: self.finish_confirmation(box, mode, answer))
            self.confirmation = box
            box.show()
        self.refresh()

    def finish_confirmation(self, box, mode, answer):
        if self.confirmation is box:
            self.confirmation = None
            if answer == QtWidgets.QMessageBox.Yes:
                self.request_step('calibrate', mode, confirmed=True)
            else:
                self.refresh()

    def cancel_pending(self):
        self.scan_timer.stop()
        self.pending_step = None
        if self.scan is not None:
            self.scan.cancel()
        if self.confirmation is not None:
            box, self.confirmation = self.confirmation, None
            box.close()
        if self.target_dialog is not None:
            dialog, self.target_dialog = self.target_dialog, None
            dialog.reject()

    def stop_all(self):
        self.cancel_pending()
        if self.cancel_pending_goal is not None:
            self.cancel_pending_goal()
        self.controller.stop_all()
        self.refresh()

    def refresh(self, now=None):
        immediate = now is None
        now = time.monotonic() if now is None else now
        states = self.controller.states
        pending_key = self.pending_step[0] if self.pending_step is not None else None
        calibration_blocked = (pending_key in ('calibrate', 'map') or self.confirmation is not None
                               or self.controller.calibrating or self.controller.switching)
        if calibration_blocked != self.calibration_blocked:
            self.calibration_blocked = calibration_blocked
            self.calibration_active.emit(calibration_blocked)
        for key, button in self.buttons.items():
            raw_color, raw_suffix, clickable = states[key]
            color, suffix = self.stable.update(key, (raw_color, raw_suffix), now,
                                               immediate=immediate or not clickable or key == 'vision')
            if key == pending_key:
                color, suffix = 'yellow', '检查中'
            if button.property('stage') != color:
                button.setProperty('stage', color)
                button.style().unpolish(button)
                button.style().polish(button)
            suffix = suffix or ('' if key == 'wgs84' and color == 'red' else STATUS[color])
            tail = ' · ' + suffix if suffix else ''
            title = ('3. 释放' if key == 'vision' and (suffix in ('抓取完成', '需释放') or suffix.startswith('释放'))
                     else STEPS[key])
            if key == 'wgs84' and suffix == '取消本地任务':
                title, tail = suffix, ''
            text = title + tail
            metrics = button.fontMetrics()
            room = button.width() - metrics.horizontalAdvance(tail) - 15
            rendered = metrics.elidedText(title, QtCore.Qt.ElideRight, max(0, room)) + tail
            if button.text() != rendered:
                button.setText(rendered)
            if button.toolTip() != text:
                button.setToolTip(text)
                button.setAccessibleName(text)
            if button.isEnabled() != clickable:
                button.setEnabled(clickable)
        for key, action in self.diagnostics_actions.items():
            _, suffix, clickable = states[key]
            action.setText(DIAGNOSTICS[key] + (' · ' + suffix if suffix else ''))
            action.setEnabled(clickable and key != pending_key)
        self.air_mode.setEnabled(pending_key != 'air' and states['air'][0] == 'red' and states['air'][2])
        messages = list(self.controller.messages)[-3:]
        text = '\n'.join(str(line).strip().replace('\n', ' ') for line in messages)
        if self.log.toPlainText() != text:
            self.log.setPlainText(text)
            self.log.setToolTip(text)
            self.log.moveCursor(QtGui.QTextCursor.End)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self.refresh()
