# Workspace2 无RTK在线建图导航（雷达版）

本项目从原`slicer_navigation_without_mapping`迁移，运行不挂载、不source、不写入旧`Slicer_Workspace`。保留ROS Noetic算法栈，未改为ROS2/Nav2。

## 保留的链路

MID360 → FAST-LIO → LIO二维投影＋轮编码器/EKF → Gmapping在线/map → Dijkstra网格搜索＋原B样条全局平滑插件 → TEB → 原速度平滑/前方停车线 → 原速度复用器 → Workspace2同款SLC2 0x16车体闭环。

不加载离线地图，不启动map_server、AMCL、RTK或GNSS。按本次要求，已移除深度相机驱动、点云处理、避障层与RViz相机显示。雷达保留360°扫描、车体自过滤、地面高度过滤和原障碍标记/射线清除策略。

保留多点巡航、出发朝向调整、位置进入容差即锁定、单独yaw对齐、原后退恢复/重试/跳点、故障零速等待、地图重置及目标坐标窗口。最小转弯半径仍0；未增加二次包络或新的速度补偿。

## 启动

```bash
cd /home/potato/Slicer_Workspace2/src/Online_Navigation_Without_RTK
./run.sh build
./run.sh check
./run.sh nav --background
# 无图形界面
# ./run.sh nav --background gui:=false
```

启动本身不发目标、不自动巡航。先退出Workspace2/Test的WASD/手柄和其他导航实例，同一C30D只允许一个串口所有者。此入口只管理`workspace2-online-navigation`，不会停止通信或旧导航容器。没有自动刷写固件。

仅建图/看雷达，不打开底盘串口：
```bash
./run.sh nav --background start_control:=false
```

RViz Fixed Frame=map，等在线地图出现后用2D Nav Goal点击并拖动朝向。青色矩形与红色X轴显示车体位置/朝向。没有AMCL，不需要2D Pose Estimate。

```bash
# 本次地图的绝对目标，米/弧度
./run.sh goal --x 1.0 --y 0.3 --yaw 0.0
# 发送时车头正前方1米
./run.sh goal --relative --x 1.0 --y 0.0 --yaw 0.0
# 录点，不发运动目标；RViz使用Record Waypoint工具
./run.sh record
# 编辑config/waypoints.yaml后，才手动开始多点巡航
./run.sh patrol
./run.sh diagnose --duration 30
./run.sh goal-ui
./run.sh reset-map
./run.sh stop
```

路点文件实际位置：`catkin_ws/src/slicer_navigation_without_mapping/config/waypoints.yaml`（根目录config中有快捷链接）。新一轮从空白SLAM启动后，地图原点重新建立，旧路点不能当作永久世界坐标。查看点击位置但不发导航目标：`docker exec -it workspace2-online-navigation bash -c 'source /opt/ros/noetic/setup.bash; rostopic echo /clicked_point'`，然后在RViz选择Publish Point。

## 配置位置与实测几何

- `config/slicer_config.yaml`：车体、雷达外参、速度与footprint入口。
- `config/waypoints.yaml`：本次地图的多点目标。
- `catkin_ws/src/slicer_navigation_without_mapping/config/nav_params.yaml`：在线场景TEB覆盖项。
- `catkin_ws/src/slicer_navigation/config/teb_local_planner_params.yaml`：原TEB优化权重。
- `catkin_ws/src/slicer_navigation/config/navigation_params.yaml`：原Dijkstra与B样条参数。
- `catkin_ws/src/slicer_navigation_without_mapping/config/runtime.yaml`：Docker镜像、ROS Master、ROS_IP。
- `../../Hardware/config.yaml`：Test同源的设备身份、通信、编码器/电机极性、输出百分比；导航不会覆盖该文件。

本次确认：轮径65mm；轴距130mm；两侧轮内缘间距152mm，每轮宽27mm，因此轮中心距179mm、轮胎总外宽206mm。雷达相对旋转中心为`[0.065,0,0.095] m`，姿态仍沿用原水平安装外参。全车载荷的前后包络未重新测量，规划仍保留原保守包络与3cm边距，不把轮胎外形误当作完整载荷边界。

最低识别高度：`config/slicer_config.yaml → sensors.scan_min_height: 0.05`，单位米，相对base_link地面基准；不是相对雷达中心。修改后重新启动本项目，生成到`.runtime/.generated/scan.yaml`。不要把二维LaserScan层的min_obstacle_height一起改成正5cm，二维投影的Z=0。

原线速度上限0.6m/s、原加速度、原距离/角度容差继承；TEB在线覆盖角速度1.2rad/s。不是速度最低值，拐弯/避障/终点仍可减速。

## 新底盘适配

ROS适配包保持`slicer_wasd_control`名称以兼容原launch和话题，内部改为直接import新工作空间`Hardware/Host/chassis.py`，与`Test/`使用相同串口独占、SLC2身份确认、0x12极性、0x16 vx/wz和0x18输出上限接口。

目标新板序列号`5CF0016129`；PWM极性`[1,-1,-1,-1]`，编码器极性`[1,-1,1,-1]`。不使用旧板6377/6379的身份或极性。100%输出百分比仍代表下位机80%PWM上限，不把速度命令改成PWM。

`/wheel/odom`的平移使用累计编码器差分、偏航使用下位机IMU角速度反馈；EKF继续独占`odom→base_link`，Gmapping独占`map→odom`。驱动不重复发布TF。MCU故障时维持零速，Motor OFF后才按原流程清锁，旧指令丢弃，避免自动重放。

注意：本次未修改/重刷已验证v1.1固件。该固件内的差速前馈几何仍是轮距0.15m、轴距0.17m，而新实测为0.179m、0.13m；有效和差约3.4%。导航配置和ROS里程计已使用新尺寸，下位机IMU闭环仍在，但若要把下位机前馈尺寸也精确同步，需要单独更新固件并按断动力电流程刷写。本迁移不隐式执行这一步。

## 目录

- `catkin_ws/src/slicer_navigation_without_mapping/`：雷达版在线SLAM、目标、重置、显示和巡航入口。
- `catkin_ws/src/slicer_navigation/`：原B样条插件、TEB/巡航算法、速度出口与雷达过滤。
- `catkin_ws/src/slicer_bringup/`：原传感器TF、LIO投影、EKF。
- `catkin_ws/src/fast_lio/`、`livox_ros_driver2/`：独立复制并重新编译的雷达源代码。
- `catkin_ws/src/slicer_wasd_control/`：新底盘ROS适配＋原速度复用器。
- `config/templates/`：原生成参数的迁移快照，启动时只在本项目生成运行参数。
- `migration/`：参考源码哈希、旧目录隔离核验与验证结果。
- `logs/`：编译和隔离检查日志。

原生依赖使用Jetson基础`slicer-fkie:latest`镜像派生的`workspace2-online-navigation:local`镜像（ROS Noetic、PCL、Livox SDK2、gmapping、robot_localization、TEB等），不依赖旧工程的devel/build产物。构建并发为1，减少首次编译内存峰值；运行时保留原轻量显示/CPU设置。

`check`使用network=none、无设备映射、独立ROS Master：验证真实Gmapping、B样条、TEB和雷达costmap，使用合成扫描与假底盘验证非空规划/非零命令、障碍清除、地图重置和新驱动故障恢复。实车传感器、实际转向和制动效果仍需现场验证；本次不自动启动实车导航。

## 2026-11-08 离线验收

独立catkin构建、19项Python检查、B样条几何检查、目标坐标UI检查、真实Gmapping/TEB隔离集成、281项完整实机launch参数解析均通过。测试未映射任何设备，未启动真实舵机或底盘；旧目录8614项mode/size/mtime/ctime/链接目标与迁移前一致。详细记录：`migration/validation.json`、`migration/isolation_report.json`、`logs/build.log`、`logs/check.log`。

## 启动依赖修复

原基础镜像缺少psmisc/fuser，真实串口驱动在检查占用时会报`No such file or directory: fuser`，进而触发required节点退出。现在由`docker/Dockerfile`补齐运行依赖；没有跳过fuser检查，没有删除TIOCEXCL/flock，也没有修改控制算法。

如需重建专用镜像：
```bash
docker build --network host -f docker/Dockerfile -t workspace2-online-navigation:local docker
```
`runtime.yaml`选择该镜像。`check`额外调用真实device选择函数，在临时普通文件上验证空闲/占用分支，不打开任何物理串口。

本依赖修复已通过21项Python检查及隔离导航集成。实机采用独立11341 Master、无GUI/无目标，25秒内底盘ready=true、483帧轮里程计、251帧扫描、13次地图更新，498条cmd_vel均为零。测试实例已停止；未做实车行驶验证。详见`migration/startup_dependency_fix.json`。
