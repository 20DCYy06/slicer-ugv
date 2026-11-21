#!/usr/bin/env python3
"""Jetson desktop companion for the existing RTK navigation stack."""
from pathlib import Path
import signal
import sys
import time

from PyQt5 import QtCore, QtGui, QtWidgets
try:
    from PyQt5 import sip
except ImportError:
    import sip
import rosgraph
import rospy
from rviz import bindings as rviz

from dashboard_model import Dashboard, split_screen
from ros_observer import Observer


STYLES = """
QWidget#Dashboard { background: #101923; color: #e9f3fc; }
QLabel { color: #e9f3fc; }
QLabel[muted="true"] { color: #a3b6c7; }
QFrame#Readouts { background: #172534; border: 1px solid #2c4051; border-radius: 8px; }
QProgressBar { background: #293e50; border: 0; border-radius: 3px; }
QProgressBar::chunk { background: #59c4ec; border-radius: 3px; }
QToolBar#DesktopTools { background: #172534; padding: 5px; border-bottom: 1px solid #344d60; spacing: 7px; }
QToolBar#DesktopTools QToolButton { color: #e9f3fc; padding: 7px 12px; background: #243a4b; border-radius: 4px; }
QToolBar#DesktopTools QToolButton:checked { background: #79cfed; color: #0a2030; }
"""
COLORS = {
    'good': '#61f0a8', 'caution': '#ffdc74', 'warn': '#ffac63',
    'bad': '#ff8585', 'neutral': '#a3b6c7',
}
WINDOW_FLAGS = QtCore.Qt.Window | QtCore.Qt.FramelessWindowHint | QtCore.Qt.WindowStaysOnTopHint


def label(text='', muted=False):
    widget = QtWidgets.QLabel(text)
    widget.setTextFormat(QtCore.Qt.PlainText)
    widget.setProperty('muted', muted)
    widget.setWordWrap(True)
    return widget


class Readouts(QtWidgets.QWidget):
    def __init__(self):
        super().__init__(flags=WINDOW_FLAGS)
        self.setObjectName('Dashboard')
        self.setWindowTitle('RTK 实时状态')
        self.setStyleSheet(STYLES)
        outer = QtWidgets.QVBoxLayout(self)
        outer.setContentsMargins(18, 15, 18, 15)
        outer.setSpacing(12)
        self.rtk = label()
        outer.addWidget(self.rtk)
        self.status = label('', True)
        outer.addWidget(self.status)
        self.sections = QtWidgets.QStackedWidget()
        outer.addWidget(self.sections, 1)

        self.prep = QtWidgets.QFrame(objectName='Readouts')
        prep = QtWidgets.QVBoxLayout(self.prep)
        self.phase = label('地理配准')
        self.registration = label()
        self.progress = QtWidgets.QProgressBar(minimum=0, maximum=4, textVisible=False)
        self.progress.setFixedHeight(7)
        prep.addWidget(self.phase)
        prep.addWidget(self.registration)
        prep.addWidget(self.progress)
        self.prep_goal = label()
        prep.addWidget(self.prep_goal)
        prep.addStretch()
        self.system = {}
        for name, title in (('lidar', 'MID360 / 扫描'), ('lio', 'LIO'), ('driver', '底盘驱动'), ('mux', 'Mux')):
            self.system[name] = title, label()
            prep.addWidget(self.system[name][1])
        prep.addStretch()
        self.sections.addWidget(self.prep)

        nav = QtWidgets.QFrame(objectName='Readouts')
        rows = QtWidgets.QVBoxLayout(nav)
        self.destination = label()
        self.own = label()
        rows.addWidget(self.destination)
        rows.addWidget(self.own)
        self.points = [label() for _ in range(3)]
        for item in self.points:
            rows.addWidget(item)
        rows.addStretch()
        self.sections.addWidget(nav)
        self.feedback = label('', True)
        self.feedback.hide()
        outer.addWidget(self.feedback)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        size = min(19, max(11, min(self.width() // 25, self.height() // 30)))
        self.setFont(QtGui.QFont('Noto Sans CJK SC', size))
        self.rtk.setFont(QtGui.QFont('Noto Sans CJK SC', size + 2, QtGui.QFont.Bold))
        self.destination.setFont(QtGui.QFont('Noto Sans CJK SC', size + 1, QtGui.QFont.Bold))

    def refresh(self, view):
        self.rtk.setText(view['rtk'])
        self.rtk.setStyleSheet('color: ' + COLORS[view['fix_color']])
        status = ('新鲜 RTK 可用' if view['usable'] else '等待新鲜 RTK') + '  |  ' + view['frame']
        if view['degraded']:
            status += '  |  配准降级'
        self.status.setText(status)
        self.sections.setCurrentIndex(int(view['navigation']))
        self.registration.setText(view['registration'])
        self.progress.setValue(view['progress'])
        self.prep_goal.setText(view['destination'])
        self.prep_goal.setVisible(view['destination'] == '正在前往：RViz手动目标')
        for key, (title, widget) in self.system.items():
            text, color = view[key]
            widget.setText(f'{title}　{text}')
            widget.setStyleSheet('color: ' + COLORS[color])
        self.destination.setText(view['destination'])
        self.own.setText(view['own'])
        for widget, text in zip(self.points, view['points']):
            widget.setText(text)
        self.feedback.setText(view['feedback'])
        self.feedback.setVisible(bool(view['feedback']))


class DesktopRviz(rviz.VisualizationFrame):
    def closeEvent(self, event):
        # Geometry and Fixed Frame are session state. Keep the preset reusable.
        event.accept()


def make_rviz(config_path, app):
    window = DesktopRviz()
    window.setSplashPath('')
    window.initialize()
    config = rviz.Config()
    reader = rviz.YamlConfigReader()
    reader.readFile(config, str(config_path))
    if reader.error():
        raise RuntimeError(reader.errorMessage())
    window.load(config)
    window.setWindowTitle('RTK 导航 RViz[*]')
    window.setWindowFlags(WINDOW_FLAGS)
    window.setHideButtonVisibility(False)
    for old_toolbar in window.findChildren(QtWidgets.QToolBar):
        old_toolbar.hide()
    # RViz keeps internal pointers to menus until its docks are destroyed.
    window.menuBar().hide()
    manager = window.getManager()
    tools = manager.getToolManager()
    toolbar = QtWidgets.QToolBar('导航交互', window, objectName='DesktopTools')
    toolbar.setStyleSheet(STYLES)
    toolbar.setMovable(False)
    window.addToolBar(toolbar)
    group = QtWidgets.QActionGroup(toolbar)
    group.setExclusive(True)
    move_tool = tools.getTool(0)
    waypoint_tool = tools.getTool(2)

    def select(tool):
        tools.setDefaultTool(waypoint_tool if tool == waypoint_tool else move_tool)
        tools.setCurrentTool(tool)

    buttons = []
    for index, text in enumerate(('移动视图', '2D Nav Goal', 'Waypoint 打点')):
        tool = tools.getTool(index)
        action = toolbar.addAction(text)
        action.setCheckable(True)
        group.addAction(action)
        action.triggered.connect(lambda _checked, item=tool: select(item))
        buttons.append((tool, action))
        if index == 0:
            action.setChecked(True)
    toolbar.addSeparator()
    toolbar.addAction('关闭', app.quit)
    refresh_tools = QtCore.QTimer(toolbar)

    def mark_current_tool():
        current = tools.getCurrentTool()
        for tool, button in buttons:
            if current == tool:
                button.setChecked(True)
                break

    refresh_tools.timeout.connect(mark_current_tool)
    refresh_tools.start(120)
    select(move_tool)
    return window, manager


def place_windows(screen, frame, dashboard):
    area = screen.geometry()
    left, right = split_screen(area.x(), area.y(), area.width(), area.height())
    frame.setGeometry(*left)
    dashboard.setGeometry(*right)


def main():
    if not rosgraph.is_master_online():
        raise RuntimeError('ROS master 未连接。请先启动现有 RTK 定位或导航入口。')
    rospy.init_node('slicer_rtk_desktop', anonymous=True, disable_signals=True)
    app = QtWidgets.QApplication(sys.argv[:1])
    board = Dashboard()
    observer = Observer(board)
    dashboard = Readouts()
    frame, manager = make_rviz(Path(__file__).with_name('rtk_desktop.rviz'), app)

    def tile():
        place_windows(app.primaryScreen(), frame, dashboard)

    app.primaryScreen().geometryChanged.connect(lambda _geometry: tile())
    frame.show()
    dashboard.show()
    tile()
    QtCore.QTimer.singleShot(0, tile)
    shortcuts = [QtWidgets.QShortcut(QtGui.QKeySequence('Escape'), parent)
                 for parent in (frame, dashboard)]
    for shortcut in shortcuts:
        shortcut.activated.connect(app.quit)
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: app.quit())

    def refresh():
        if rospy.is_shutdown() or not frame.isVisible() or not dashboard.isVisible():
            app.quit()
            return
        view = observer.view(time.monotonic())
        if manager.getFixedFrame() != view['frame']:
            manager.setFixedFrame(view['frame'])
        dashboard.refresh(view)

    timer = QtCore.QTimer()
    timer.timeout.connect(refresh)
    timer.start(150)
    try:
        return app.exec_()
    finally:
        timer.stop()
        manager.stopUpdate()
        frame.hide()
        sip.delete(frame)
        dashboard.close()
        rospy.signal_shutdown('RTK desktop closed')


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (RuntimeError, OSError) as error:
        print(f'RTK 桌面：{error}', file=sys.stderr)
        raise SystemExit(2)
