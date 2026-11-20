#!/usr/bin/env python3
"""隔离网络/无串口：真实 RViz 插件鼠标连续点击 -> ROS -> 临时 YAML -> 控制台。

需在 Noetic Docker 中连接 Jetson X11（不用虚拟 framebuffer 或额外依赖）。
测试仅操作自己的嵌入式 RViz 窗口，不向用户现有 RViz 注入鼠标事件。
"""
import json
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import xmlrpc.client
import yaml

if os.environ.get('SLICER_ISOLATED_GUI_TEST') != '1':
    raise RuntimeError('Only run in an isolated --network none Docker, with no hardware devices')
os.environ['DISABLE_ROS1_EOL_WARNINGS'] = '1'

from PyQt5 import QtCore, QtWidgets, QtTest
try:
    from PyQt5 import sip
except ImportError:
    import sip
from rviz import bindings as rviz
import rospy
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String

PKG = Path(__file__).resolve().parents[1]
app = QtWidgets.QApplication([])
children = []
frame = None


def wait_for(predicate, timeout=12):
    deadline = time.monotonic()+timeout
    while time.monotonic() < deadline:
        app.processEvents()
        if predicate():
            return
        time.sleep(.03)
    raise AssertionError('GUI recording test timed out')


try:
    with tempfile.TemporaryDirectory(prefix='record-tool-gui-') as directory:
        root = Path(directory)
        with open(root/'console.log', 'w') as output:
            master = subprocess.Popen(['roscore'], stdout=output, stderr=subprocess.STDOUT)
            children.append(master)
            proxy = xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI'])
            def ready():
                try:
                    return proxy.getPid('/record_tool_test')[0] == 1
                except OSError:
                    return False
            wait_for(ready)
            rospy.init_node('record_tool_gui_test', disable_signals=True)
            target = root/'waypoints.yaml'
            target.write_text('frame_id: map\nwaypoints:\n-\n')
            recorder = subprocess.Popen(['rosrun','slicer_navigation','waypoint_recorder.py',
                '_file:='+str(target)], stdout=output, stderr=subprocess.STDOUT)
            children.append(recorder)
            statuses, clicks = [], []
            subs = [rospy.Subscriber('/slicer/waypoint_record/status', String,
                                    lambda m: statuses.append(json.loads(m.data))),
                    rospy.Subscriber('/clicked_point', PointStamped, lambda m: clicks.append(m))]
            wait_for(lambda: any(s['state']=='READY' for s in statuses))
            frame = rviz.VisualizationFrame()
            frame.setSplashPath('')
            frame.initialize()
            frame.resize(1000,700)
            manager = frame.getManager()
            manager.setFixedFrame('map')
            # 只放一个网格，不放 Map/PointCloud，证明录点不依赖渲染实体深度拾取。
            manager.createDisplay('rviz/Grid','Test grid',True)
            manager.getViewManager().setCurrentViewControllerType('rviz/TopDownOrtho')
            tool = manager.getToolManager().addTool('slicer_navigation/RecordWaypoint')
            manager.getToolManager().setCurrentTool(tool)
            frame.show()
            widgets = frame.findChildren(QtWidgets.QWidget)
            panels = [w for w in widgets if 'RenderPanel' in w.metaObject().className()]
            assert panels, [w.metaObject().className() for w in widgets]
            panel = panels[0]
            wait_for(lambda: panel.width()>100 and panel.height()>100)
            # 等 ROS 订阅握手和首次渲染。
            for _ in range(30):
                app.processEvents(); time.sleep(.05)
            for index in range(10):
                QtTest.QTest.mouseClick(panel, QtCore.Qt.LeftButton, QtCore.Qt.NoModifier,
                    QtCore.QPoint(panel.width()//2+index*4,panel.height()//2+20))
                app.processEvents()
                QtTest.QTest.qWait(60)
            wait_for(lambda: len([s for s in statuses if s['state']=='SAVED'])==10)
            saved = yaml.safe_load(target.read_text())['waypoints']
            assert len(clicks)==len(saved)==10, (len(clicks),len(saved))
            assert all(next(iter(p))=='name' for p in saved)
            assert manager.getToolManager().getCurrentTool().getClassId()=='slicer_navigation/RecordWaypoint'
            output.flush()
            console = (root/'console.log').read_text()
            assert console.count('Received map click')==10
            assert console.count('Recorded point_')==10
            assert 'Recorded point_010 (#10)' in console
            assert not any(s['state']=='ERROR' for s in statuses)
            frame.close()
            sip.delete(frame)
            frame = None
            app.processEvents()
            print('RVIZ_REAL_MOUSE_10_OF_10_SAVED_AND_PRINTED_PASS', flush=True)
finally:
    if frame is not None:
        frame.close()
        sip.delete(frame)
        app.processEvents()
    if rospy.core.is_initialized():
        rospy.signal_shutdown('isolated GUI test complete')
    for child in reversed(children):
        if child.poll() is None:
            child.send_signal(signal.SIGINT)
            try:
                child.wait(timeout=5)
            except subprocess.TimeoutExpired:
                child.kill(); child.wait()
