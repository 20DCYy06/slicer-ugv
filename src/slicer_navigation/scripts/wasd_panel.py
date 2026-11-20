#!/usr/bin/env python3
"""建图/导航中的 WASD 窗口：只发 /teleop/cmd_vel，复用现有驱动与仲裁。"""
import signal
import sys
import time
from pathlib import Path
import rospkg
import rospy
from geometry_msgs.msg import Twist
from PyQt5 import QtCore, QtGui, QtWidgets

WASD = Path(rospkg.RosPack().get_path('slicer_wasd_control'))
sys.path.insert(0, str(WASD / 'scripts'))
from control_core import Keys, load_config


class WasdPanel(QtWidgets.QWidget):
    def __init__(self, config, publisher):
        super().__init__()
        self.config, self.publisher = config, publisher
        self.keys = Keys(config)
        self.held, self.was_active = '', False
        self.setWindowTitle('Slicer WASD · 建图 / 导航手动接管')
        self.setFocusPolicy(QtCore.Qt.StrongFocus)
        self.setFont(QtGui.QFont(QtWidgets.QApplication.font().family(), 18))
        layout = QtWidgets.QVBoxLayout(self)
        layout.setSizeConstraint(QtWidgets.QLayout.SetFixedSize)
        label = QtWidgets.QLabel('按住 W/S 前后行驶，A/D 原地差速自转\n松键、切换窗口或关闭窗口即停车；空格急停')
        layout.addWidget(label)
        grid = QtWidgets.QGridLayout()
        for key, text, row, col in [('w','W 前进',0,1),('a','A 左转',1,0),('s','S 后退',1,1),('d','D 右转',1,2)]:
            button = QtWidgets.QPushButton(text)
            button.setMinimumSize(150,64)
            button.setFocusPolicy(QtCore.Qt.NoFocus)
            button.pressed.connect(lambda k=key: self.press(k))
            button.released.connect(lambda k=key: self.release(k))
            grid.addWidget(button,row,col)
        layout.addLayout(grid)
        stop = QtWidgets.QPushButton('停车 / SPACE')
        stop.setMinimumHeight(50)
        stop.setFocusPolicy(QtCore.Qt.NoFocus)
        stop.clicked.connect(self.stop)
        layout.addWidget(stop)
        scale = QtWidgets.QHBoxLayout()
        for text, char in [('减速 −','-'),('加速 +','+')]:
            button = QtWidgets.QPushButton(text)
            button.setFocusPolicy(QtCore.Qt.NoFocus)
            button.clicked.connect(lambda unused=False,k=char: self.keys.feed(k.encode(),time.monotonic()))
            scale.addWidget(button)
        layout.addLayout(scale)
        self.status = QtWidgets.QLabel('等待操作；不占用电机串口')
        self.status.setMinimumWidth(460)
        layout.addWidget(self.status)
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.tick)
        self.timer.start(round(1000/config['publish_hz']))

    def press(self, key):
        self.held = key

    def release(self, key):
        if self.held == key:
            self.stop()

    def stop(self):
        self.held = ''
        self.keys.key = ''
        if self.was_active:
            self.publisher.publish(Twist())
        self.was_active = False

    def tick(self):
        if rospy.is_shutdown():
            self.close()
            return
        now = time.monotonic()
        if self.held:
            self.keys.feed(self.held.encode(),now)
        v,w = self.keys.velocity(now)
        active = bool(v or w)
        if active or self.was_active:
            msg = Twist();msg.linear.x=v;msg.angular.z=w
            self.publisher.publish(msg)
        self.was_active=active
        self.status.setText('按键: %s  vx=%.3f  wz=%.3f  档位×%.1f\n%s' % (
            self.held.upper() or '停止',v,w,self.keys.scale,
            '已接入速度仲裁' if self.publisher.get_num_connections() else '等待 /teleop/cmd_vel 仲裁订阅'))

    def keyPressEvent(self,event):
        if event.isAutoRepeat():
            return
        key=event.text().lower()
        if key in ('w','a','s','d'):
            self.press(key)
        elif event.key() in (QtCore.Qt.Key_Space,QtCore.Qt.Key_Escape):
            self.stop()
        elif key in ('+','=','-'):
            self.keys.feed(key.encode(),time.monotonic())
        elif key=='q':
            self.close()
        else:
            super().keyPressEvent(event)

    def keyReleaseEvent(self,event):
        if not event.isAutoRepeat():
            self.release(event.text().lower())

    def event(self,event):
        if event.type()==QtCore.QEvent.WindowDeactivate and hasattr(self,'keys'):
            self.stop()
        return super().event(event)

    def closeEvent(self,event):
        self.stop()
        self.timer.stop()
        event.accept()


def main():
    rospy.init_node('slicer_wasd_panel',disable_signals=True)
    config=load_config(rospy.get_param('~config_file',str(WASD/'config/teleop_params.yaml')))
    publisher=rospy.Publisher(config['teleop_topic'],Twist,queue_size=1)
    app=QtWidgets.QApplication(sys.argv[:1])
    # 复用宿主机中文字体；镜像中不联网安装字体或桌面组件。
    for filename in ('/opt/host-fonts/opentype/noto/NotoSansCJK-Regular.ttc',
                     '/tmp/slicer_CJK_font.ttc','/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'):
        if Path(filename).is_file():
            font_id=QtGui.QFontDatabase.addApplicationFont(filename)
            families=QtGui.QFontDatabase.applicationFontFamilies(font_id)
            if families:
                family='Noto Sans CJK SC' if 'Noto Sans CJK SC' in families else families[0]
                app.setFont(QtGui.QFont(family,18))
                break
    window=WasdPanel(config,publisher)
    for signum in (signal.SIGINT,signal.SIGTERM):
        signal.signal(signum,lambda *unused: window.close())
    window.show()
    rospy.loginfo('WASD panel publishes %s; shared chassis driver/mux',config['teleop_topic'])
    try:
        return app.exec_()
    finally:
        window.stop()
        rospy.signal_shutdown('WASD panel closed')


if __name__=='__main__':
    sys.exit(main())
