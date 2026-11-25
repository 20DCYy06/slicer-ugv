# RTK MID360 体素障碍候选

独立分支为 `feat/rtk-lidar-voxel`，基于 RTK 路线连续性及固定地图修复。车端候选为 `~/workspace/slicer/releases/20261125-rtk-voxel-rc1`。以该目录的 `release.json` 核对安装对应的完整源码版本。候选软件已通过隔离测试，现场点云、控制负载与绕障效果待验收。

## 感知与地图

- 候选从 FAST-LIO 的 `/cloud_registered_body` 取得最近一帧去畸变点云。发布帧为 IMU 坐标 `body`，时间戳为该帧雷达结束时刻。已有过滤节点保留原 `/cloud_nav_filtered → /scan`，并从同一帧另发 `/cloud_nav_voxel`。体素输入移除车身矩形加既有 padding 内的回波、现有机械臂包络内的高点和非有限值。Raytrace 起点显式使用物理 `laser_link`，ROS 以原始测量时刻把起点和点云分别变换到相应 costmap frame。
- `lidar_voxel.yaml` 在 RTK 全局与局部图各用一个官方 `costmap_2d::VoxelLayer` 和原有膨胀层。RTK 准备阶段沿用 `odom`，完成后全局图用 `map`、局部图用 `odom`。二维地图话题与 Guard 接口保留。Guard 继续读 `/scan`。独立 NoRTK、Gmapping 保留原障碍层。若显式开启深度相机，独立相机层仍接在最后一次膨胀之前。
- 纵向配置以上车静态安装参数中的地面投影为参考。激光原点约高 0.18 m，`origin_z=0`、16 层 × 0.05 m 覆盖 `[0,0.80)` m。障碍标记取 0.05–0.50 m 回波，与现有 scan 的低障碍门槛一致。清除使用 −0.15 至 0.80 m 的实测回波，射线进入体素范围后才清除，较低地面回波也可提供净空证据。任何一个占据体素即可让对应二维格成为障碍（`mark_threshold=0`）。空列保持未知，至少一个体素有净空证据时才可投影为已观察空闲（`unknown_threshold=15`）。标记和清除的水平量程沿用 4.5 m 配置。候选保留有限测量，未见回波的低障碍和坡地通行能力仍需真车确认。
- 已安装 Noetic 的 VoxelLayer 按高度体素清除，二维投影继续供原规划和控制链消费。三维网格本身不发布。90 m 全局图新增约 13 MB 体素数组，其余开销和短时 CPU/RSS 以车端实际测量为准。相关实现见 [Noetic VoxelLayer](https://github.com/ros-planning/navigation/blob/1.17.3/costmap_2d/plugins/voxel_layer.cpp) 和 [ObservationBuffer](https://github.com/ros-planning/navigation/blob/1.17.3/costmap_2d/src/observation_buffer.cpp)。

## 核对与对照

现场负责人结束原导航、雷达和控制会话并确认静止后，按[维护手册](../MAINTENANCE.md)选择候选发布。保留原 `current` 目标以便恢复，再执行以下命令核对并启动桌面。

```bash
BASE="$HOME/workspace/slicer"
"$BASE/run" status
SLICER_RTK_LIDAR_MODE=voxel "$BASE/run" rtk
```

在桌面依次启动 RTK、雷达与底盘、导航。启动后可用 `run shell rosparam get /ugv/move_base/global_costmap/plugins` 和 `run shell rosparam get /ugv/move_base/local_costmap/plugins` 确认两图都用 `voxel_layer`。观察 `/cloud_nav_voxel` 与两张二维图，在候选前先完成 scan 的基础路线，比较时保持同一目标、起点及起步转向开关。

在开跑前另开终端，用相同发布的 shell 启动短录包。测试结束后用 Ctrl+C 关闭录包。

```bash
BASE="$HOME/workspace/slicer"
mkdir -p "$BASE/data/rtk-voxel"
"$BASE/run" shell rosbag record -O "$BASE/data/rtk-voxel/$(date +%Y%m%d-%H%M%S).bag" \
  /cloud_registered_body /cloud_nav_voxel /scan /tf /tf_static /odom /lio/odom /wheel/odom \
  /ugv/move_base/global_costmap/costmap /ugv/move_base/global_costmap/costmap_updates \
  /ugv/move_base/local_costmap/costmap /ugv/move_base/local_costmap/costmap_updates \
  /ugv/move_base/GlobalPlanner/plan_raw /ugv/move_base/GlobalPlanner/plan \
  /ugv/move_base/GlobalPlanner/smoothing_status /ugv/move_base/DWAPlannerROS/local_plan \
  /ugv/move_base/goal /ugv/move_base/status /ugv/move_base/result \
  /geographic/goal /geographic/goal_result /geographic/status /geographic/navigation_runtime \
  /cmd_vel/nav_raw /cmd_vel/nav /cmd_vel /slicer_nav_arc_guard/hold_reason \
  /cup_nav/manual_state /diagnostics
```

回退时用桌面“一键全部停止”，退出该候选桌面，等导航与传感器退出。用 `SLICER_RTK_LIDAR_MODE=scan "$BASE/run" rtk` 重新启动同一发布的二维 scan 档，或按维护手册把 `current` 换回原发布后再启动。两种方式都需要完整重建导航图与观测源。

## 软件验证

- 机械臂过滤单测 2 项通过，覆盖 scan 原输出、车身与机械臂回波移除、非有限输入与地面回波。用独立生成配置运行 RTK launch 检查 15 项通过，覆盖两级体素层、准备阶段、默认 scan、Gmapping 和深度开关。
- `tools/rtk/check_lidar_voxel_ros.py` 启动专用 loopback ROS Master，仅启动实际自体过滤器和系统安装的 `costmap_2d_node`。真实插件接受合成 `PointCloud2` 后确认低障碍被标记、漏观测不擦除、高射线不擦除低障碍、实测低射线可清除、射线之外保持未知。证据见 `~/workspace/slicer/dev/rtk-lidar-voxel/logs/voxel-synthetic.log` 与 `voxel-synthetic/`。
- 11 月 25 日的绕行录包无法提供原始三维回波。下一步在同一路线补采真实单帧点云，比较两图的标记、清除、负载和低速闭环行为。
