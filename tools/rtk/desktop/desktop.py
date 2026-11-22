#!/usr/bin/env python3
"""Jetson desktop companion for the existing RTK navigation stack."""
from concurrent.futures import ThreadPoolExecutor
import fcntl
import os
from pathlib import Path
import re
import signal
import subprocess
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

from dashboard_model import Dashboard, StableStatus, split_screen
from ros_observer import Observer
from workflow import OwnedProcess, Workflow
from workflow_discovery import discover, get_master_uri, master_local, process_snapshot, vision_has_camera
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


class ReadoutLine(QtWidgets.QLabel):
    """Keep one row and expose any clipped text in its tooltip."""
    def __init__(self, muted=False):
        super().__init__()
        self.value = ''
        self.setTextFormat(QtCore.Qt.PlainText)
        self.setProperty('muted', muted)
        self.setSizePolicy(QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Fixed)

    def set_value(self, value):
        self.value = value
        rendered = self.fontMetrics().elidedText(value, QtCore.Qt.ElideRight, self.width())
        if rendered != self.text():
            self.setText(rendered)
        self.setToolTip(value if rendered != value else '')
        self.setAccessibleName(value)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self.set_value(self.value)


class Readouts(QtWidgets.QWidget):
    def __init__(self, workflow=None, cancel_pending_goal=None):
        super().__init__(flags=WINDOW_FLAGS | QtCore.Qt.WindowStaysOnTopHint)
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
        self.workflow = WorkflowPanel(workflow, cancel_pending_goal) if workflow is not None else None
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
            readout.set_value(readout.value)

    def refresh(self, view, now=None):
        now = time.monotonic() if now is None else now
        stable = self.stable
        fix = stable.update('fix', view['fix_color'], now)
        if fix == view['fix_color']:
            self.rtk.set_value(view['rtk'])
            self.rtk.setStyleSheet('color: ' + COLORS[fix])
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
            widget.setStyleSheet('color: ' + COLORS[color])
        self.destination.set_value(view['destination'])
        own_available = not view['own'].endswith('—')
        if stable.update('own_available', own_available, now) == own_available:
            self.own.set_value(view['own'])
        self.feedback.set_value(view['feedback'])
        if self.workflow is not None:
            self.workflow.refresh(now)


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


def vision_window():
    """The standalone Tk program has an X11 window instead of a ROS image topic."""
    try:
        options = dict(capture_output=True, text=True, errors='replace', timeout=1., check=True,
                       env=dict(os.environ, LC_ALL='C'))
        listing = subprocess.run(('xwininfo', '-root', '-tree'), **options).stdout
        found = re.search(r'^\s*(0x[0-9a-f]+) "Visual Grasp[^\n]*', listing, re.MULTILINE)
        if found is not None:
            attributes = subprocess.run(('xwininfo', '-id', found[1], '-stats'), **options).stdout
            if 'Map State: IsViewable' in attributes:
                size = dict(re.findall(r'^\s*(Width|Height):\s+(\d+)\s*$', attributes, re.MULTILINE))
                return int(found[1], 16), int(size.get('Width', 640)), int(size.get('Height', 480))
    except (OSError, subprocess.SubprocessError):
        pass
    return None


def desktop_probe():
    graph = discover()
    visual = vision_window()
    return graph, visual, visual is not None and vision_has_camera(graph.processes.get('vision'))


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
    observer = Observer(board)
    workflow = Workflow(board, local_scan=process_snapshot)
    dashboard = Readouts(workflow, observer.cancel_pending_goal)
    frame, manager = make_rviz(Path(__file__).with_name('rtk_desktop.rviz'), app)
    executor = ThreadPoolExecutor(max_workers=1)
    probe = executor.submit(desktop_probe)
    next_probe = 0.
    placed_vision = None

    def tile():
        nonlocal placed_vision
        place_windows(app.primaryScreen(), frame, dashboard)
        placed_vision = None

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
        nonlocal probe, next_probe, placed_vision
        if rospy.is_shutdown() or not frame.isVisible() or not dashboard.isVisible():
            app.quit()
            return
        now = time.monotonic()
        if probe is not None and probe.done():
            graph, visual, camera_open = probe.result()
            workflow.update(graph, vision=camera_open)
            visual_id = visual[0] if visual is not None else None
            if visual is not None and visual_id != placed_vision:
                _, natural_width, natural_height = visual
                preview = QtGui.QWindow.fromWinId(visual_id)
                if preview is not None:
                    area = frame.geometry()
                    width = min(natural_width, max(300, area.width() - 36))
                    height = min(natural_height, max(220, area.height() - 84))
                    preview.setGeometry(area.x() + 12, area.y() + 52, width, height)
                    frame.lower()
                    preview.requestActivate()
                    dashboard.raise_()
            placed_vision, probe = visual_id, None
        else:
            workflow.update()
        if probe is None and now >= next_probe:
            probe = executor.submit(desktop_probe)
            next_probe = now + .55
        view = observer.view(now)
        # RViz and the goal dispatcher use the same observed planner frame.
        frame_id = board.planner_frame(now)
        if frame_id is not None and manager.getFixedFrame() != frame_id:
            manager.setFixedFrame(frame_id)
        dashboard.refresh(view, now)

    timer = QtCore.QTimer()
    timer.timeout.connect(refresh)
    timer.start(150)
    try:
        return app.exec_()
    finally:
        timer.stop()
        observer.cancel_pending_goal()
        workflow.stop_all()
        while not workflow.idle:
            workflow.update()
            time.sleep(.1)
        executor.shutdown()
        manager.stopUpdate()
        frame.hide()
        sip.delete(frame)
        dashboard.close()
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
