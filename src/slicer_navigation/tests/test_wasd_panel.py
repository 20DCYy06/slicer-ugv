#!/usr/bin/env python3
"""无串口、无ROS master：检查按住、松键、失焦、关闭时的手动控制语义。"""
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from PyQt5 import QtCore,QtGui,QtWidgets
from wasd_panel import WasdPanel,load_config,WASD


class Publisher:
    def __init__(self):self.messages=[]
    def publish(self,msg):self.messages.append((msg.linear.x,msg.angular.z))
    def get_num_connections(self):return 1


app=QtWidgets.QApplication([])
pub=Publisher()
panel=WasdPanel(load_config(WASD/'config/teleop_params.yaml'),pub)
panel.keyPressEvent(QtGui.QKeyEvent(QtCore.QEvent.KeyPress,QtCore.Qt.Key_W,QtCore.Qt.NoModifier,'w'))
for _ in range(80):panel.tick()
assert pub.messages[-1][0]>0 and pub.messages[-1][1]==0
panel.keyReleaseEvent(QtGui.QKeyEvent(QtCore.QEvent.KeyRelease,QtCore.Qt.Key_W,QtCore.Qt.NoModifier,'w'))
assert pub.messages[-1]==(0,0)
panel.press('a');panel.tick()
assert pub.messages[-1][0]==0 and pub.messages[-1][1]>0
panel.event(QtCore.QEvent(QtCore.QEvent.WindowDeactivate))
assert pub.messages[-1]==(0,0)
panel.press('s');panel.tick()
assert pub.messages[-1][0]<0
panel.close()
assert pub.messages[-1]==(0,0)
print('WASD_PANEL_PRESS_RELEASE_FOCUS_PASS',flush=True)
