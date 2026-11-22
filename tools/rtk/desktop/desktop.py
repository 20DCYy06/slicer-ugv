#!/usr/bin/env python3
"""Jetson desktop companion for the existing RTK navigation stack."""
from concurrent.futures import ThreadPoolExecutor
import fcntl
import os
from pathlib import Path
import signal
import sys
import time
from urllib.parse import urlparse

from PyQt5 import QtCore, QtGui, QtWidgets
try:
    from PyQt5 import sip
except ImportError:
    import sip
import rospy
from rviz import bindings as rviz

from dashboard_model import Dashboard, StableStatus
from map_display import MapDisplay
from ros_observer import Observer
from workflow import OwnedProcess, Workflow
from workflow_discovery import Graph, discover, get_master_uri, master_local, process_snapshot
from workflow_panel import WorkflowPanel


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
WINDOW_FLAGS = QtCore.Qt.Window | QtCore.Qt.FramelessWindowHint
DISPLAY_INTERVAL_MS = 250
WORKFLOW_INTERVAL_MS = 150
PROBE_INTERVAL_S = .9
LOG_INTERVAL_S = .5


class ReadoutLine(QtWidgets.QLabel):
    """Keep one row and expose any clipped text in its tooltip."""
    def __init__(self, muted=False):
        super().__init__()
        self.value = ''
        self.setTextFormat(QtCore.Qt.PlainText)
        self.setProperty('muted', muted)
        self.setSizePolicy(QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Fixed)

    def set_value(self, value):
        if value == self.value:
            return
        self.value = value
        self.setAccessibleName(value)
        self.update_text()

    def set_color(self, color):
        style = 'color: ' + COLORS[color]
        if style != self.styleSheet():
            self.setStyleSheet(style)

    def update_text(self):
        rendered = self.fontMetrics().elidedText(self.value, QtCore.Qt.ElideRight, self.width())
        if rendered != self.text():
            self.setText(rendered)
        tooltip = self.value if rendered != self.value else ''
        if tooltip != self.toolTip():
            self.setToolTip(tooltip)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self.update_text()


class Readouts(QtWidgets.QWidget):
    def __init__(self, workflow=None, cancel_pending_goal=None, submit_process_scan=None):
        super().__init__()
        self.setObjectName('Dashboard')
        self.setWindowTitle('RTK 实时状态')
        self.setStyleSheet(STYLES)
        self.stable = StableStatus()
        outer = QtWidgets.QVBoxLayout(self)
        outer.setContentsMargins(12, 10, 12, 10)
        outer.setSpacing(5)
        self.rtk = ReadoutLine()
        self.status = ReadoutLine(muted=True)
        outer.addWidget(self.rtk)
        outer.addWidget(self.status)

        readouts = QtWidgets.QScrollArea(widgetResizable=True)
        readouts.setFrameShape(QtWidgets.QFrame.NoFrame)
        readouts.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Ignored)
        readouts.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        page = QtWidgets.QFrame(objectName='Readouts')
        readouts.setWidget(page)
        outer.addWidget(readouts, 1)
        rows = QtWidgets.QVBoxLayout(page)
        self.destination = ReadoutLine()
        self.own = ReadoutLine()
        self.registration = ReadoutLine()
        self.progress = QtWidgets.QProgressBar(minimum=0, maximum=4, textVisible=False)
        self.progress.setFixedHeight(7)
        for item in (self.destination, self.own, self.registration, self.progress):
            rows.addWidget(item)
        self.system = {}
        for name, title in (('lidar', 'MID360 扫描'), ('lio', 'LIO'), ('driver', '底盘驱动'), ('mux', 'Mux')):
            self.system[name] = title, ReadoutLine()
            rows.addWidget(self.system[name][1])
        rows.addStretch()

        self.feedback = ReadoutLine(muted=True)
        outer.addWidget(self.feedback)
        self.workflow = (WorkflowPanel(workflow, cancel_pending_goal, submit_process_scan)
                         if workflow is not None else None)
        if self.workflow is not None:
            outer.addWidget(self.workflow)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        available = self.height() - (self.workflow.sizeHint().height() if self.workflow else 0)
        size = min(22, max(8, min(self.width() // 20, available // 23)))
        family = 'Noto Sans CJK SC'
        coordinates_example = '正在前往：-90.0000000, -180.0000000'
        coordinate_size = size
        # Reserve space for the two layouts and the optional vertical scrollbar.
        while coordinate_size > 6:
            font = QtGui.QFont(family, coordinate_size, QtGui.QFont.Bold)
            if QtGui.QFontMetrics(font).horizontalAdvance(coordinates_example) <= self.width() - 64:
                break
            coordinate_size -= 1
        rtk_size = size + 2
        rtk_example = 'RTK状态：SINGLE（卫星：999 颗）'
        while rtk_size > 8:
            font = QtGui.QFont(family, rtk_size, QtGui.QFont.Bold)
            if QtGui.QFontMetrics(font).horizontalAdvance(rtk_example) <= self.width() - 24:
                break
            rtk_size -= 1
        for readout in self.findChildren(ReadoutLine):
            if readout in (self.destination, self.own):
                point_size = coordinate_size
            else:
                point_size = rtk_size if readout is self.rtk else size
            weight = QtGui.QFont.Bold if readout in (self.rtk, self.destination) else QtGui.QFont.Normal
            readout.setFont(QtGui.QFont(family, point_size, weight))
            readout.setFixedHeight(readout.fontMetrics().height() + 2)
            readout.update_text()

    def refresh(self, view, now=None):
        now = time.monotonic() if now is None else now
        stable = self.stable
        fix = stable.update('fix', view['fix_color'], now)
        if fix == view['fix_color']:
            self.rtk.set_value(view['rtk'])
            self.rtk.set_color(fix)
        status = ('新鲜 RTK 可用' if view['usable'] else '等待新鲜 RTK') + '  |  ' + view['frame']
        if view['degraded']:
            status += '  |  配准降级'
        self.status.set_value(stable.update('status', status, now))
        self.registration.set_value(stable.update('registration', view['registration'], now,
                                                  immediate=view['collecting']))
        self.progress.setValue(stable.update('progress', view['progress'], now))
        for key, (title, widget) in self.system.items():
            value = view[key]
            immediate = (key == 'driver' and value[1] == 'bad'
                         or key == 'mux' and value[1] in ('bad', 'caution'))
            message, color = stable.update(key, value, now, immediate)
            widget.set_value(f'{title}　{message}')
            widget.set_color(color)
        self.destination.set_value(view['destination'])
        own_available = not view['own'].endswith('—')
        if stable.update('own_available', own_available, now) == own_available:
            self.own.set_value(view['own'])
        self.feedback.set_value(view['feedback'])
        if self.workflow is not None:
            self.workflow.refresh(now)


class DesktopRviz(rviz.VisualizationFrame):
    closing = QtCore.pyqtSignal()
    readouts_dock = None

    def attach_readouts(self, readouts):
        dock = QtWidgets.QDockWidget('RTK 实时状态', self, objectName='RTKReadouts')
        dock.setAllowedAreas(QtCore.Qt.RightDockWidgetArea)
        dock.setFeatures(QtWidgets.QDockWidget.NoDockWidgetFeatures)
        title = QtWidgets.QWidget(dock)
        title.setFixedHeight(0)
        dock.setTitleBarWidget(title)
        dock.setWidget(readouts)
        self.readouts_dock = dock
        self.addDockWidget(QtCore.Qt.RightDockWidgetArea, dock)
        self.size_readouts()

    def size_readouts(self):
        if self.readouts_dock is not None:
            self.readouts_dock.setFixedWidth(min(max(215, self.width() // 4), self.width() // 3))

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self.size_readouts()

    def closeEvent(self, event):
        # Geometry and Fixed Frame are session state. Keep the preset reusable.
        event.accept()
        self.closing.emit()


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
    for dock in window.findChildren(QtWidgets.QDockWidget, options=QtCore.Qt.FindDirectChildrenOnly):
        dock.hide()
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
                if button.isChecked():
                    break
                button.setChecked(True)
                break

    refresh_tools.timeout.connect(mark_current_tool)
    refresh_tools.start(DISPLAY_INTERVAL_MS)
    select(move_tool)
    return window, manager


def ensure_master():
    if discover().online:
        return None
    if not master_local():
        raise RuntimeError('远端 ROS Master 未连接，请检查 ROS_MASTER_URI。')
    env = os.environ.copy()
    env.pop('ROS_NAMESPACE', None)
    port = urlparse(get_master_uri()).port or 11311
    child = OwnedProcess('master', ('roscore', '-p', str(port)), env)
    deadline = time.monotonic() + 10.
    try:
        while time.monotonic() < deadline:
            if discover().online:
                return child if child.poll() is None else None
            if child.poll() is not None:
                break
            time.sleep(.1)
        message = child.lines()
        raise RuntimeError('本机 ROS Master 启动失败。' + (' ' + message[-1] if message else ''))
    except BaseException:
        stop_master(child, startup_failed=True)
        raise


def stop_master(child, desktop_node='', startup_failed=False):
    if child is None:
        return
    # An external terminal can join a master bootstrapped by this desktop.
    graph = discover()
    if not startup_failed and (not graph.online or graph.nodes - {'/rosout', desktop_node}):
        print('保留 ROS Master，避免中断外部节点。', flush=True)
        return
    child.stop()
    while child.poll() is None:
        time.sleep(.1)


def run_desktop():
    rospy.init_node('slicer_rtk_desktop', anonymous=True, disable_signals=True)
    app = QtWidgets.QApplication(sys.argv[:1])
    board = Dashboard()
    map_display = MapDisplay()
    observer = Observer(board)
    workflow = Workflow(board)
    executor = ThreadPoolExecutor(max_workers=2)
    dashboard = Readouts(workflow, observer.cancel_pending_goal, lambda: executor.submit(process_snapshot))
    frame, manager = make_rviz(Path(__file__).with_name('rtk_desktop.rviz'), app)
    frame.closing.connect(app.quit)
    frame.attach_readouts(dashboard)
    dashboard.workflow.calibration_active.connect(observer.set_calibration_active)
    probe = executor.submit(discover)
    next_probe = time.monotonic() + PROBE_INTERVAL_S
    next_logs = 0.
    screen = app.primaryScreen()

    def place_window():
        frame.setGeometry(screen.availableGeometry())

    screen.availableGeometryChanged.connect(lambda _geometry: place_window())
    place_window()
    frame.show()
    shortcut = QtWidgets.QShortcut(QtGui.QKeySequence('Escape'), frame)
    shortcut.activated.connect(app.quit)
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: app.quit())

    def service_workflow():
        nonlocal probe, next_probe, next_logs
        if rospy.is_shutdown():
            app.quit()
            return
        now = time.monotonic()
        read_logs = now >= next_logs
        if read_logs:
            next_logs = now + LOG_INTERVAL_S
        if probe is not None and probe.done():
            try:
                graph = probe.result()
            except Exception as error:
                workflow.report(f'桌面状态读取失败，将重试。{error}')
                graph = Graph()
            workflow.update(graph, read_logs=read_logs)
            probe = None
        else:
            workflow.update(read_logs=read_logs)
        if probe is None and now >= next_probe:
            probe = executor.submit(discover)
            next_probe = now + PROBE_INTERVAL_S

    def refresh_display():
        now = time.monotonic()
        view = observer.view(now)
        # RViz and the goal dispatcher use the same observed planner frame.
        frame_id = board.planner_frame(now)
        if frame_id is not None and manager.getFixedFrame() != frame_id:
            manager.setFixedFrame(frame_id)
        dashboard.refresh(view, now)

    workflow_timer = QtCore.QTimer(interval=WORKFLOW_INTERVAL_MS)
    workflow_timer.timeout.connect(service_workflow)
    workflow_timer.start()
    display_timer = QtCore.QTimer(interval=DISPLAY_INTERVAL_MS)
    display_timer.timeout.connect(refresh_display)
    display_timer.start()
    try:
        return app.exec_()
    finally:
        display_timer.stop()
        workflow_timer.stop()
        dashboard.workflow.cancel_pending()
        manager.stopUpdate()
        frame.hide()
        observer.cancel_pending_goal()
        workflow.stop_all()
        while not workflow.idle:
            workflow.update()
            time.sleep(.1)
        executor.shutdown()
        sip.delete(frame)
        rospy.signal_shutdown('RTK desktop closed')


def main():
    runtime = Path(os.environ.get('XDG_RUNTIME_DIR') or f'/tmp/slicer-rtk-{os.getuid()}')
    runtime.mkdir(mode=0o700, exist_ok=True)
    with (runtime / 'slicer-rtk-desktop.lock').open('a') as instance:
        try:
            fcntl.flock(instance, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as error:
            raise RuntimeError('已有本地 RTK 桌面窗口。') from error
        child = ensure_master()
        try:
            return run_desktop()
        finally:
            stop_master(child, rospy.get_name())


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (RuntimeError, OSError) as error:
        print(f'RTK 桌面：{error}', file=sys.stderr)
        raise SystemExit(2)
