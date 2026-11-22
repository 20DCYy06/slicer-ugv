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

from dashboard_model import Dashboard, split_screen
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


def label(text='', muted=False):
    widget = QtWidgets.QLabel(text)
    widget.setTextFormat(QtCore.Qt.PlainText)
    widget.setProperty('muted', muted)
    widget.setWordWrap(True)
    return widget


class Readouts(QtWidgets.QWidget):
    def __init__(self, workflow=None):
        super().__init__(flags=WINDOW_FLAGS | QtCore.Qt.WindowStaysOnTopHint)
        self.setObjectName('Dashboard')
        self.setWindowTitle('RTK 实时状态')
        self.setStyleSheet(STYLES)
        outer = QtWidgets.QVBoxLayout(self)
        outer.setContentsMargins(12, 10, 12, 10)
        outer.setSpacing(5)
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
        self.workflow = WorkflowPanel(workflow) if workflow is not None else None
        if self.workflow is not None:
            outer.addWidget(self.workflow)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        available = self.height() - (self.workflow.sizeHint().height() if self.workflow else 0)
        size = min(22, max(9, min(self.width() // 20, available // 23)))
        body_font = QtGui.QFont('Noto Sans CJK SC', size)
        for readout in self.findChildren(QtWidgets.QLabel):
            if self.workflow is None or not self.workflow.isAncestorOf(readout):
                readout.setFont(body_font)
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
        if self.workflow is not None:
            self.workflow.refresh()


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
    dashboard = Readouts(workflow)
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
        # The live action parameter supersedes any late costmap from the outgoing planner.
        frame_id = workflow.graph.frame or view['frame']
        if manager.getFixedFrame() != frame_id:
            manager.setFixedFrame(frame_id)
        dashboard.refresh(view)

    timer = QtCore.QTimer()
    timer.timeout.connect(refresh)
    timer.start(150)
    try:
        return app.exec_()
    finally:
        timer.stop()
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
