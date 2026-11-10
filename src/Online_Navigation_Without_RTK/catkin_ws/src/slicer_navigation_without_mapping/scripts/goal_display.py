#!/usr/bin/env python3
"""桌面目标坐标：只订阅move_base实际接受的目标，不发目标或速度。"""
import math
import signal
import sys
from PyQt5 import QtCore, QtGui, QtWidgets


class GoalWindow(QtWidgets.QWidget):
    pose_received=QtCore.pyqtSignal(object)
    map_reset=QtCore.pyqtSignal()

    def __init__(self):
        super().__init__()
        self.setWindowTitle('导航目标坐标')
        self.setWindowFlag(QtCore.Qt.WindowStaysOnTopHint,True)
        self.label=QtWidgets.QLabel()
        self.label.setTextFormat(QtCore.Qt.PlainText)
        self.label.setFont(QtGui.QFont('Sans Serif',22))
        self.label.setStyleSheet('color: #111; background: white; padding: 14px;')
        layout=QtWidgets.QVBoxLayout(self);layout.addWidget(self.label)
        self.resize(370,220)
        self.pose_received.connect(self.show_pose)
        self.map_reset.connect(self.clear_pose)
        self.clear_pose()

    @QtCore.pyqtSlot()
    def clear_pose(self):
        self.label.setText('目标坐标\nX: — m\nY: — m\nYaw: — rad')

    @QtCore.pyqtSlot(object)
    def show_pose(self,message):
        p=message.pose.position;q=message.pose.orientation
        yaw=math.atan2(2.*(q.w*q.z+q.x*q.y),1.-2.*(q.y*q.y+q.z*q.z))
        self.label.setText(f'{message.header.frame_id}\nX: {p.x:.3f} m\nY: {p.y:.3f} m\nYaw: {yaw:.3f} rad')


def main():
    import rospy
    from geometry_msgs.msg import PoseStamped
    from std_msgs.msg import Empty
    rospy.init_node('online_goal_coordinates',disable_signals=True)
    app=QtWidgets.QApplication(sys.argv[:1]);window=GoalWindow()
    # ROS回调在线程中运行；通过Qt信号更新界面，不跨线程操作控件。
    subscribers=[rospy.Subscriber('/move_base/current_goal',PoseStamped,window.pose_received.emit,queue_size=1),
                 rospy.Subscriber('/online_navigation/map_reset',Empty,lambda m:window.map_reset.emit(),queue_size=1)]
    timer=QtCore.QTimer();timer.timeout.connect(lambda:app.quit() if rospy.is_shutdown() else None);timer.start(200)
    for signum in (signal.SIGINT,signal.SIGTERM):signal.signal(signum,lambda *_:app.quit())
    window.show()
    area=app.primaryScreen().availableGeometry();window.move(area.right()-window.width()-16,area.top()+40)
    result=app.exec_();rospy.signal_shutdown('目标坐标窗口关闭')
    return result


if __name__=='__main__':sys.exit(main())
