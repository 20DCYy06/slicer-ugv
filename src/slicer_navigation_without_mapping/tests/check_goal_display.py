#!/usr/bin/env python3
"""离屏检查坐标窗口，不连接ROS Master，不发送目标或电机指令。"""
import math
from pathlib import Path
import sys
from PyQt5 import QtWidgets
from geometry_msgs.msg import PoseStamped
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from goal_display import GoalWindow

app=QtWidgets.QApplication([]);window=GoalWindow()
pose=PoseStamped();pose.header.frame_id='map';pose.pose.position.x=2.5857;pose.pose.position.y=-.0571
pose.pose.orientation.z=math.sin(.75/2);pose.pose.orientation.w=math.cos(.75/2)
window.pose_received.emit(pose);app.processEvents()
assert window.label.text()=='map\nX: 2.586 m\nY: -0.057 m\nYaw: 0.750 rad'
window.map_reset.emit();app.processEvents()
assert '—' in window.label.text() and '2.586' not in window.label.text()
window.close()
print('GOAL_COORDINATES_UI_PASS: metric coordinates, yaw radians, reset clears old coordinates',flush=True)
