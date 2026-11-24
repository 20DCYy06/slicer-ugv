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
from actionlib_msgs.msg import GoalID, GoalStatus, GoalStatusArray
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseStamped, TransformStamped
from move_base_msgs.msg import MoveBaseActionGoal
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Bool, String

DIRECTORY = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(DIRECTORY))
from desktop import Dashboard, Observer, Readouts, make_rviz, sip
from ros_observer import WAYPOINT_CLICK
from workflow import Workflow


def main():
    if not rospy.get_param('/rtk_desktop_isolated_test', False):
        raise RuntimeError('Only run against a dedicated ROS master marked for this test')
    rospy.init_node('rtk_desktop_test', disable_signals=True)
    app = QtWidgets.QApplication([])
    observer = Observer(Dashboard())
    dashboard = Readouts(Workflow(observer.board))
    dashboard.bind_waypoint_confirmation(observer)
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
    runtime_pub = rospy.Publisher('/geographic/navigation_runtime', String, queue_size=1)
    v2_pub = rospy.Publisher('/air_ground/v2/state', String, queue_size=1)
    gnss = rospy.Publisher('/gnss/diagnostics', DiagnosticArray, queue_size=1)
    raw_fix = rospy.Publisher('/gnss/fix', NavSatFix, queue_size=1)
    raw_quality = rospy.Publisher('/gnss/fix_quality', DiagnosticArray, queue_size=1)
    gnss_pair = 'unmatched'
    gnss_session = 'desktop-test'
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
    runtime = {'phase': 'IDLE', 'busy': False}
    route = {'phase': 'IDLE'}

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
        if gnss_pair != 'silent':
            fixed = gnss_pair != 'lost'
            stamp = now - rospy.Duration(.01) if gnss_pair == 'unmatched' else now
            raw_quality.publish(DiagnosticArray(header=dict_header(stamp), status=[
                DiagnosticStatus(name='cup_gnss/receiver', values=[
                    KeyValue('gga_quality', '4' if fixed else '5'),
                    KeyValue('fresh_rtk_usable', 'true' if fixed else 'false'),
                    KeyValue('fix_valid', 'true'), KeyValue('session', gnss_session),
                ]),
            ]))
            if fixed:
                raw_fix.publish(NavSatFix(header=dict_header(now, 'gnss_link'),
                                          status=NavSatStatus(status=NavSatStatus.STATUS_GBAS_FIX),
                                          latitude=31.21, longitude=121.51, altitude=18.))
        runtime_pub.publish(json.dumps(dict(runtime, stamp=now.to_sec())))
        v2_pub.publish(json.dumps(dict(route, updated_at=now.to_sec())))
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
        assert observer.board.fixed_position(time.monotonic()) is None
        gnss_pair = 'matched'
        wait(lambda: dashboard.own.value == '当前位置：31.2100000, 121.5100000')
        assert observer.board.fixed_position(time.monotonic())['altitude'] == 18.
        assert not any(row.text().startswith('RTK 天线') for row in dashboard.findChildren(QtWidgets.QLabel))
        dashboard.own.actions()[0].trigger()
        assert QtWidgets.QApplication.clipboard().text() == '31.2100000, 121.5100000'
        gnss_pair = 'lost'
        wait(lambda: dashboard.own.value == '当前位置：—')
        assert observer.board.fixed_position(time.monotonic()) is None
        gnss_pair = 'matched'
        wait(lambda: dashboard.own.value == '当前位置：31.2100000, 121.5100000')
        gnss_session = 'desktop-test-reconnected'
        gnss_pair = 'unmatched'
        wait(lambda: dashboard.own.value == '当前位置：—')
        gnss_pair = 'matched'
        wait(lambda: dashboard.own.value == '当前位置：31.2100000, 121.5100000')
        gnss_pair = 'silent'
        wait(lambda: dashboard.own.value == '当前位置：—')
        assert observer.board.fixed_position(time.monotonic()) is None
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

        def click(tool, at=position):
            choose[tool].trigger()
            QtTest.QTest.mouseClick(panel, QtCore.Qt.LeftButton, QtCore.Qt.NoModifier, at)

        with redirect_stdout(output):
            click('Waypoint 打点')
            wait(lambda: len(clicks) == 1 and '等待地理配准' in output.getvalue()
                 and dashboard.workflow.waypoint_dialog is not None)
            dashboard.workflow.waypoint_dialog.reject()
            assert clicks[0].header.frame_id == 'odom' and not goals
            # A brief Action heartbeat gap must not reject an otherwise valid click.
            status_delay = .7
            wait(lambda: observer.board.recent('action', time.monotonic(), .5) is None
                 and observer.board.recent('action', time.monotonic(), 1.5) is not None)
            click('2D Nav Goal')
            wait(lambda: observer.pending_goal is not None)
            first_pending = observer.pending_goal
            click('2D Nav Goal', position - QtCore.QPoint(30, 0))
            wait(lambda: observer.pending_goal is not None and observer.pending_goal is not first_pending)
            latest_click = observer.pending_goal[0]
            assert not goals
            status_delay = 0.
            wait(lambda: len(goals) == 1)
            assert goals[0].goal.target_pose.header.frame_id == 'odom'
            assert goals[0].goal.target_pose.pose.position == latest_click.pose.position
            assert dashboard.destination.value == '正在前往：待配准或等待 TF'
            active[:] = [GoalStatus(goal_id=goals[0].goal_id, status=GoalStatus.ACTIVE)]
            wait(lambda: observer.board.recent('action', time.monotonic())
                 == [(goals[0].goal_id.id, GoalStatus.ACTIVE)])
            click('2D Nav Goal', position + QtCore.QPoint(50, 0))
            wait(lambda: len(goals) == 2)
            assert goals[1].goal_id.id != goals[0].goal_id.id
            assert goals[1].goal.target_pose.pose.position != goals[0].goal.target_pose.pose.position
            active[0].status = GoalStatus.PREEMPTED
            active.append(GoalStatus(goal_id=goals[1].goal_id, status=GoalStatus.ACTIVE))
            foreign = MoveBaseActionGoal(goal_id=GoalID(id='external-goal'), goal=goals[1].goal)
            observer.action_goal(foreign)
            click('2D Nav Goal')
            wait(lambda: output.getvalue().count('当前目标优先') == 1)
            assert len(goals) == 2
            active.append(GoalStatus(goal_id=GoalID(id='external-goal'), status=GoalStatus.ACTIVE))
            wait(lambda: (observer.board.recent('action', time.monotonic()) or [None])[-1]
                 == ('external-goal', GoalStatus.ACTIVE))
            assert observer.goal_id == goals[1].goal_id.id
            click('2D Nav Goal')
            wait(lambda: output.getvalue().count('当前目标优先') == 2)
            assert len(goals) == 2
            active[1].status = GoalStatus.SUCCEEDED
            active.pop()
            runtime.update(phase='PREPARING', busy=True, distance_m=2.4, span_m=1.7, rmse_m=.06,
                           target={'latitude': 31.1, 'longitude': 121.1}, reason='paired RTK')
            route.update(task_id='inflight', phase='READY', target={'lat': 31.3, 'lon': 121.4})
            temporary = MoveBaseActionGoal(goal_id=GoalID(id='temporary-odom'), goal=goals[1].goal)
            temporary._connection_header = {'callerid': '/slicer_navigation_runtime'}
            observer.action_goal(temporary)
            assert 'temporary-odom' not in observer.action_goals
            active.append(GoalStatus(goal_id=temporary.goal_id, status=GoalStatus.ACTIVE))
            wait(lambda: dashboard.destination.value == '正在前往：31.3000000, 121.4000000'
                 and '自动准备运动' in dashboard.task_phase.value
                 and '路程 2.4 m · 展开 1.7 m · 残差 0.06 m' == dashboard.task_metrics.value)
            click('2D Nav Goal')
            wait(lambda: '正在自动准备地理航行' in output.getvalue())
            assert len(goals) == 2
            route.update(phase='STOPPED')
            runtime.update(phase='IDLE', busy=False)
            active.pop()
            wait(lambda: dashboard.destination.value == '正在前往：待命')
            rospy.set_param('/ugv/move_base/global_costmap/global_frame', 'map')
            wait(lambda: observer.board.recent('configured_frame', time.monotonic()) == 'map')
            assert manager.getFixedFrame() == 'odom'
            grid_frame = 'map'
            wait(lambda: manager.getFixedFrame() == 'map'
                 and re.fullmatch(r'当前位置：31\.\d{7}, 121\.\d{7}', dashboard.own.value))
            click('Waypoint 打点')
            wait(lambda: len(clicks) == 2 and re.search(
                r'经纬度：3[01]\.\d{7}, 121\.\d{7}\n', output.getvalue()))
            wait(lambda: dashboard.workflow.waypoint_dialog is not None)
            dialog = dashboard.workflow.waypoint_dialog
            assert dashboard.waypoint_coords in dialog.text()
            assert clicks[1].header.frame_id == 'map' and len(goals) == 2
            dialog.reject()
            observer.set_calibration_active(True)
            click('2D Nav Goal')
            wait(lambda: '正在地理配准' in output.getvalue())
            assert len(goals) == 2
            observer.set_calibration_active(False)
            runtime.update(phase='BLOCKED', reason='previous WGS84 preparation blocked')
            wait(lambda: dashboard.destination.value == '正在前往：31.1000000, 121.1000000')
            held = True
            wait(lambda: (observer.board.recent('mux', time.monotonic()) or {}).get('manual_hold') is True)
            click('2D Nav Goal')
            wait(lambda: 'Mux 保持停车' in output.getvalue())
            assert len(goals) == 2
            held = False
            wait(lambda: (observer.board.recent('mux', time.monotonic()) or {}).get('manual_hold') is False)
            click('2D Nav Goal')
            wait(lambda: len(goals) == 3)
            assert goals[2].goal.target_pose.header.frame_id == 'map'
            active[:] = [GoalStatus(goal_id=goals[2].goal_id, status=GoalStatus.ACTIVE)]
            wait(lambda: dashboard.destination.value == '正在前往：31.2000000, 121.5000000'
                 and dashboard.task_phase.value == '任务阶段：导航中')
            runtime.update(phase='IDLE')
            assert ' X:' not in output.getvalue()
            target_row = dashboard.destination.geometry()
            dashboard.feedback.set_value('等待' * 200)
            app.processEvents()
            assert dashboard.destination.geometry() == target_row
            assert not dashboard.feedback.wordWrap() and dashboard.feedback.toolTip()
            assert dashboard.destination.text() == dashboard.destination.value
            active[:] = [GoalStatus(goal_id=goals[2].goal_id, status=GoalStatus.SUCCEEDED)]
            wait(lambda: dashboard.destination.value == '正在前往：待命')
            click('Waypoint 打点', position + QtCore.QPoint(50, 0))
            wait(lambda: len(clicks) == 3 and dashboard.workflow.waypoint_dialog is not None)
            assert len(goals) == 3
            dialog = dashboard.workflow.waypoint_dialog
            QtTest.QTest.mouseClick(dialog.button(QtWidgets.QMessageBox.Yes), QtCore.Qt.LeftButton)
            wait(lambda: len(goals) == 4)
            assert goals[3].goal.target_pose.pose.position == clicks[2].pose.position
            status_delay = 2.
            wait(lambda: observer.board.recent('action', time.monotonic(), 1.5) is None)
            click('2D Nav Goal')
            wait(lambda: '等待当前导航坐标系和 move_base 状态' in output.getvalue())
            assert len(goals) == 4
        print('RTK_DESKTOP_GUI_PASS split odom map waypoint WGS84 preparation manual goal isolation')
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
