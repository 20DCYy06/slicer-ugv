# 局部视觉对准

`vision_align.py` 显示本地图像、视频或 ROS `sensor_msgs/Image`，用鼠标选择一个点，或一条边的两个端点。点模式追踪所点像素；边模式分别追踪两端，以端点均值为特征。它不识别固定物体、颜色或尺寸；当前临时物体约 10×7×5 cm，不用于推算速度或距离。请选择有纹理的角点/端点，纯色平面会拒绝选取，跟踪丢失后需重新点击。

已确认安装：D455固定在车尾、朝下看（2026-11-10用户确认）。图像上下/左右不能直接当作底盘前后/左右，必须以该安装下的实测局部响应标定。现有Guard禁止倒车，因此首版不能保证任意像素xy均可到达；需要后退的解会明确停零并显示原因。

默认仅观察，不创建 Twist publisher。程序不打开底盘串口、不控制舵机、不启动相机或底盘、不解除 Mux hold。实车驱动、release/hold 和运动由现场操作者执行。

## 离线观察

依赖 Python3、OpenCV GUI、NumPy、PyYAML；离线模式不需要 ROS。以下命令在 `Slicer_Workspace2` 目录执行，需要可用图形桌面或 X11 转发：

```bash
python3 Test/Vision_adjust/vision_align.py --help
python3 Test/Vision_adjust/vision_align.py --image /path/to/current_image.png
python3 Test/Vision_adjust/vision_align.py --video /path/to/current_video.mp4 --feature edge
python3 Test/Vision_adjust/vision_align.py --image /path/to/current_image.png --target 320 240
```

点击一个点，或用 `--feature edge` 点击两个端点。边模式点击第一点后冻结当前画面，视频也停止读下一帧，等待在同一画面点击第二点；选取期间控制归零。第二点选完后恢复跟踪，特征所在帧必须与检查新鲜度的帧一致才能控制。窗口中的 `target X`、`target Y` 滑条随时调整目标像素；默认目标是图像中心。窗口显示目标、特征、像素误差、状态和控制速度。空格暂停/恢复，`r` 清除选择，Esc 或关闭窗口退出。视频按 GUI 刷新推进，不保证文件原始播放速度；暂停时停止读下一帧。

## ROS 图像与 RGB 相机候选

在已 source ROS Noetic 和当前独立 RTK 工程环境的终端中：

```bash
python3 Test/Vision_adjust/vision_align.py --topic /camera/color/image_raw
```

ROS 模式另需 `cv_bridge`、`sensor_msgs`，控制模式需 `geometry_msgs`。默认配置文件为同目录 `config.yaml`，可用 `--config FILE` 指定本次配置。未写 `--image`、`--video`、`--topic` 时，订阅配置的 `image_topic`，仍为观察模式。

现有 `slicer_depth.launch` 的相机分支关闭 RGB，不能据此声称彩色流已开启。以下是由操作者单独启动的 RGB 候选，未部署或启动验证；已存在相机驱动时先由操作者解决占用，不重复打开设备：

```bash
roslaunch realsense2_camera rs_camera.launch \
  enable_color:=true enable_depth:=false serial_no:=342343063024
rostopic info /camera/color/image_raw
rostopic echo -n 2 /camera/color/image_raw/header
```

这里使用不带下划线的 ROS1 序列号。已核对官方 ROS1 [nodelet.launch.xml](https://github.com/IntelRealSense/realsense-ros/blob/ros1-legacy/realsense2_camera/launch/includes/nodelet.launch.xml) 明确以 `type="str"` 设置 `serial_no`，[工厂代码](https://github.com/IntelRealSense/realsense-ros/blob/ros1-legacy/realsense2_camera/src/realsense_node_factory.cpp) 直接匹配序列号，不剥离 `_`。应先检查现场安装包是否与此一致；不要套用 ROS2 的 `_342343063024` 写法。此命令只申请 RGB，不能代替已有的深度避障输入；Guard 需要的既有感知和 costmap 仍须有效。

## 标定和可选控制

`jacobian` 默认 `null`，所以直接加 `--control` 会拒绝运行。当前现场图像、实测标定和实际闭环运动仍待验证；后续可根据本次采样协助计算并填写参数，未完成前保持观察模式。二维矩阵的定义为：

```text
J = [[Δ像素x / Δ前进米,  Δ像素x / Δ偏航弧度],
     [Δ像素y / Δ前进米,  Δ像素y / Δ偏航弧度]]
```

每列分别取另一自由度保持时的测量，使用真实前进距离/偏航角，而非命令值或物体尺寸。保留相机姿态、分辨率、特征和工作距离；不要猜摄像头安装方向及符号。测量运动由操作者执行，程序没有自动标定运动。矩阵须有限且可逆；物体深度或相机位置变化后应重新评估标定。

控制计算为 `[vx,wz] = gain × J⁻¹ × (target-feature)`；`gain` 默认 `0.5 /s`，欧氏像素误差不大于 `tolerance_px`（默认 5）时停机。上限直接读取实际 ROS 参数 `/ugv/move_base/DWAPlannerROS/max_vel_x` 与 `max_vel_theta`，两轴统一缩放，没有额外隐含速度上限。当前 Guard 禁止倒车，若求解需要负 `vx`，两轴归零并显示 `reverse_forbidden`。

控制只接受 ROS 图像。图像 `header.stamp` 必须为正且持续递增，ROS 时间中的采集年龄与本地单调时钟中的接收年龄均不得超过 `frame_timeout_s`（默认 0.25 s）；重复、倒退、未来或过期时间戳会停止输出。暂停、重选、跟踪丢失、未选特征及退出/异常都发送零速度；退出时的最终零消息仍依赖 ROS 连接正常，既有 Guard/Mux 超时机制继续负责底盘链路。

控制的唯一输出为 `/cmd_vel/nav_raw`，再经过已有 Slicer Guard/Mux。启动前检查 ROS master 的 publishers 与 `/slicer_nav_arc_guard/input_topic`，有其他 raw publisher 或 Guard 输入不匹配即拒绝创建 publisher；运行时约每秒复查一次并更新实际 DWA 上限，失败则退出并尝试最终零输出。

视觉对准时保留 `move_base` 及 costmaps，将其速度输出移至独立未用 topic，期间不发送导航目标。由操作者启动导航栈时使用以下参数；已有同名节点时不要重复启动：

```bash
roslaunch ugv_bringup slicer_odom.launch \
  local_planner:=dwa connect_control:=true \
  planner_raw_topic:=/cmd_vel/nav_plan_unused \
  source_config:="$(rospack find ugv_bringup)/config/online_runtime/config/slicer_config.yaml"
```

在填好实测 `jacobian`、确认感知/Guard/Mux 和 raw publisher 归属后，操作者才选择运行：

```bash
python3 Test/Vision_adjust/vision_align.py \
  --topic /camera/color/image_raw --feature point --control
```

## 离线检查

以下测试不连接设备、不打开相机或 GUI、不发送 ROS 消息：

```bash
PYTHONDONTWRITEBYTECODE=1 python3 -s -m unittest discover -s Test/Vision_adjust/tests -p test_vision_core.py -v
PYTHONDONTWRITEBYTECODE=1 python3 -s -m unittest discover -s Test/Vision_adjust/tests -p test_vision_app.py -v
```

`test_vision_launch.py` 需要已 source 的 ROS Noetic 环境，只解析 launch，不启动节点。软件模型与后续 ROS 验证应在无设备、`--network none` 容器的独立 master 中运行；离线通过不代表现场视觉、标定或底盘运动已通过。
