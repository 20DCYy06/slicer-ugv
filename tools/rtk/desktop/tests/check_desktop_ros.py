#!/usr/bin/env python3
"""Real Qt/RViz mouse events against a loopback-only test ROS master."""
from contextlib import redirect_stdout
import io
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import time

from PyQt5 import QtCore, QtTest, QtWidgets
import rospy
import tf2_ros
from actionlib_msgs.msg import GoalStatus, GoalStatusArray
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseStamped, TransformStamped
from move_base_msgs.msg import MoveBaseActionGoal
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import Bool, String

DIRECTORY = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(DIRECTORY))
from desktop import Dashboard, Observer, Readouts, make_rviz, sip
from ros_observer import WAYPOINT_CLICK


def main():
    if not rospy.get_param('/rtk_desktop_isolated_test', False):
        raise RuntimeError('Only run against a dedicated ROS master marked for this test')
    rospy.init_node('rtk_desktop_test', disable_signals=True)
    app = QtWidgets.QApplication([])
    observer = Observer(Dashboard())
    dashboard = Readouts()
    window, manager = make_rviz(DIRECTORY/'rtk_desktop.rviz', app)
    screen = app.primaryScreen()
    window.attach_readouts(dashboard)
    window.setGeometry(screen.availableGeometry())
    window.show()
    app.processEvents()
    goals = []
    clicks = []
    subscriptions = [
        rospy.Subscriber('/ugv/move_base/goal', MoveBaseActionGoal, goals.append),
        rospy.Subscriber(WAYPOINT_CLICK, PoseStamped, clicks.append),
    ]
    status = rospy.Publisher('/rtk_desktop_test/status_input', GoalStatusArray, queue_size=1)
    geo = rospy.Publisher('/geographic/status', String, queue_size=1)
    gnss = rospy.Publisher('/gnss/diagnostics', DiagnosticArray, queue_size=1)
    driver = rospy.Publisher('/slicer_c30d_driver/ready', Bool, queue_size=1)
    mux = rospy.Publisher('/cup_nav/manual_state', String, queue_size=1)
    odom = rospy.Publisher('/global/odom', Odometry, queue_size=1)
    costmap = rospy.Publisher('/ugv/move_base/global_costmap/costmap', OccupancyGrid, queue_size=1)
    grid_frame = 'odom'
    status_delay = 0.
    held = False
    transform = tf2_ros.TransformBroadcaster()
    origin = {'latitude': 31.2, 'longitude': 121.5, 'altitude_ellipsoid_m': 18.}
    active = []

    def tick():
        now = rospy.Time.now()
        status.publish(GoalStatusArray(header=dict_header(now - rospy.Duration(status_delay)),
                                       status_list=active))
        registered = rospy.get_param('/ugv/move_base/global_costmap/global_frame') == 'map'
        geo.publish(json.dumps(dict(state='ready' if registered else 'invalid', registered=registered,
                                    status_stamp=now.to_sec(), origin=origin, datum='WGS84')))
        gnss.publish(DiagnosticArray(header=dict_header(now), status=[
            DiagnosticStatus(name='cup_gnss/receiver', values=[
                KeyValue('gga_quality', '4'), KeyValue('satellites', '28'),
                KeyValue('fresh_rtk_usable', 'true'),
            ]),
        ]))
        driver.publish(True)
        mux.publish(json.dumps(dict(schema_version=1, manual_hold=held, emergency_stop_latched=False)))
        grid = OccupancyGrid(header=dict_header(now, grid_frame), data=[0])
        grid.info.width = grid.info.height = 1
        grid.info.resolution = .25
        grid.info.origin.orientation.w = 1.
        costmap.publish(grid)
        result = Odometry(header=dict_header(now, 'map'))
        result.pose.pose.position.x = 2.
        result.pose.pose.position.y = 3.
        odom.publish(result)
        map_to_odom = TransformStamped(header=dict_header(now, 'map'), child_frame_id='odom')
        map_to_odom.transform.translation.x = 1.
        map_to_odom.transform.rotation.w = 1.
        transform.sendTransform(map_to_odom)
        view = observer.view(time.monotonic())
        if view['frame'] != manager.getFixedFrame():
            manager.setFixedFrame(view['frame'])
        dashboard.refresh(view)
        app.processEvents()

    def wait(predicate, timeout=6.):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            tick()
            if predicate():
                return
            time.sleep(.035)
        raise AssertionError('Timed out waiting for RTK desktop state')

    output = io.StringIO()
    server = None
    try:
        rospy.set_param('/ugv/move_base/global_costmap/global_frame', 'odom')
        visible = screen.availableGeometry()
        assert window.geometry() == visible
        assert dashboard.window() is window
        assert window.dockWidgetArea(window.readouts_dock) == QtCore.Qt.RightDockWidgetArea
        assert window.readouts_dock.geometry().left() >= window.width() * .72
        assert dashboard.isVisible()
        spoof = rospy.Publisher('/ugv/move_base/status', GoalStatusArray, queue_size=1)
        for _ in range(5):
            spoof.publish(GoalStatusArray(header=dict_header(rospy.Time.now())))
            time.sleep(.04)
        assert observer.board.recent('action', time.monotonic()) is None
        spoof.unregister()
        server = subprocess.Popen([sys.executable, '-s', '-B', __file__, '--mock-planner'],
                                  env=dict(os.environ, ROS_NAMESPACE='/ugv'))
        wait(lambda: dashboard.rtk.text() == 'RTK状态：FIXED（卫星：28 颗）'
             and observer.board.recent('frame', time.monotonic()) == 'odom'
             and observer.board.recent('action', time.monotonic(), .3) is not None
             and observer.board.recent('configured_frame', time.monotonic()) == 'odom'
             and observer.goal_pub.impl.has_connection('/ugv/move_base'))
        readable = [dashboard.status, dashboard.registration,
                    *(row for _, row in dashboard.system.values())]
        assert all(row.font().pointSize() >= max(15, dashboard.rtk.font().pointSize() - 2)
                   for row in readable)
        assert dashboard.destination.font().pointSize() >= 15
        panel = next(w for w in window.findChildren(QtWidgets.QWidget)
                     if 'RenderPanel' in w.metaObject().className())
        toolbar = window.findChild(QtWidgets.QToolBar, 'DesktopTools')
        choose = {action.text(): action for action in toolbar.actions()}
        position = QtCore.QPoint(panel.width() // 2, panel.height() // 2)

        def click(tool):
            choose[tool].trigger()
            QtTest.QTest.mouseClick(panel, QtCore.Qt.LeftButton, QtCore.Qt.NoModifier, position)

        with redirect_stdout(output):
            click('Waypoint 打点')
            wait(lambda: len(clicks) == 1 and '待配准' in output.getvalue())
            assert clicks[0].header.frame_id == 'odom' and not goals
            # A brief Action heartbeat gap must not reject an otherwise valid click.
            status_delay = .7
            wait(lambda: observer.board.recent('action', time.monotonic(), .5) is None
                 and observer.board.recent('action', time.monotonic(), 1.5) is not None)
            click('2D Nav Goal')
            wait(lambda: observer.pending_goal is not None)
            assert not goals
            status_delay = 0.
            wait(lambda: len(goals) == 1)
            assert goals[0].goal.target_pose.header.frame_id == 'odom'
            assert dashboard.destination.value == '正在前往：待配准或等待 TF'
            active[:] = [GoalStatus(goal_id=goals[0].goal_id, status=GoalStatus.ACTIVE)]
            wait(lambda: observer.board.recent('action', time.monotonic())
                 == [(goals[0].goal_id.id, GoalStatus.ACTIVE)])
            click('2D Nav Goal')
            wait(lambda: '当前目标优先' in output.getvalue())
            assert len(goals) == 1
            active[0].status = GoalStatus.SUCCEEDED
            rospy.set_param('/ugv/move_base/global_costmap/global_frame', 'map')
            wait(lambda: observer.board.recent('configured_frame', time.monotonic()) == 'map')
            assert manager.getFixedFrame() == 'odom'
            grid_frame = 'map'
            wait(lambda: manager.getFixedFrame() == 'map'
                 and re.fullmatch(r'当前位置：31\.\d{7}, 121\.\d{7}', dashboard.own.value))
            click('Waypoint 打点')
            wait(lambda: len(clicks) == 2 and re.search(
                r'经纬度：3[01]\.\d{7}, 121\.\d{7}\n', output.getvalue()))
            assert clicks[1].header.frame_id == 'map' and len(goals) == 1
            observer.set_calibration_active(True)
            click('2D Nav Goal')
            wait(lambda: '正在地理配准' in output.getvalue())
            assert len(goals) == 1
            observer.set_calibration_active(False)
            held = True
            wait(lambda: (observer.board.recent('mux', time.monotonic()) or {}).get('manual_hold') is True)
            click('2D Nav Goal')
            wait(lambda: 'Mux 保持停车' in output.getvalue())
            assert len(goals) == 1
            held = False
            wait(lambda: (observer.board.recent('mux', time.monotonic()) or {}).get('manual_hold') is False)
            click('2D Nav Goal')
            wait(lambda: len(goals) == 2)
            assert goals[1].goal.target_pose.header.frame_id == 'map'
            wait(lambda: dashboard.destination.value == '正在前往：31.2000000, 121.5000000')
            assert ' X:' not in output.getvalue()
            target_row = dashboard.destination.geometry()
            dashboard.feedback.set_value('等待' * 200)
            app.processEvents()
            assert dashboard.destination.geometry() == target_row
            assert not dashboard.feedback.wordWrap() and dashboard.feedback.toolTip()
            assert dashboard.destination.text() == dashboard.destination.value
            active[:] = [GoalStatus(goal_id=goals[1].goal_id, status=GoalStatus.SUCCEEDED)]
            wait(lambda: dashboard.destination.value == '正在前往：待命')
            status_delay = 2.
            wait(lambda: observer.board.recent('action', time.monotonic(), 1.5) is None)
            click('2D Nav Goal')
            wait(lambda: '等待当前导航坐标系和 move_base 状态' in output.getvalue())
            assert len(goals) == 2
        print('RTK_DESKTOP_GUI_PASS split odom map waypoint coordinates manual goal isolation')
    except AssertionError:
        print(output.getvalue(), file=sys.stderr)
        raise
    finally:
        manager.stopUpdate()
        window.hide()
        sip.delete(window)
        for subscription in subscriptions:
            subscription.unregister()
        rospy.signal_shutdown('isolated GUI test complete')
        if server is not None:
            server.terminate()
            server.wait(timeout=5)


def dict_header(stamp, frame=''):
    from std_msgs.msg import Header
    return Header(stamp=stamp, frame_id=frame)


if __name__ == '__main__':
    if sys.argv[1:] == ['--mock-planner']:
        if not rospy.get_param('/rtk_desktop_isolated_test', False):
            raise RuntimeError('Mock planner requires an isolated test master')
        rospy.init_node('move_base')
        output = rospy.Publisher('/ugv/move_base/status', GoalStatusArray, queue_size=1)
        status_input = rospy.Subscriber('/rtk_desktop_test/status_input', GoalStatusArray, output.publish)
        goal_input = rospy.Subscriber('/ugv/move_base/goal', MoveBaseActionGoal, lambda _: None)
        rospy.spin()
    else:
        main()
