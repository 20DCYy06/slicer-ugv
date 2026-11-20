# Slicer UGV

中航工业杯整车工程。当前分支 `migration/native-noetic` 正在迁移到裸 Ubuntu 20.04 / ROS1 Noetic。车端稳定版本以现场验收记录为准。接手先读 [HANDOFF.md](HANDOFF.md) 和 [AGENTS.md](AGENTS.md)。

## 构建准备

在 Ubuntu 20.04 安装 ROS Noetic、Catkin、rosdep 和 [依赖清单](docs/DEPENDENCIES.md) 中的 Livox SDK2。WSL Jammy 可以运行源码预检。使用 Focal 环境做完整 ROS 构建。

```bash
bash tools/check_native.sh --static
NATIVE_WS="$HOME/slicer_noetic_dev"
bash tools/build_native.sh --prepare-only "$NATIVE_WS"
source /opt/ros/noetic/setup.bash
rosdep install --from-paths "$NATIVE_WS/src" --ignore-src --rosdistro noetic -r -y
bash tools/build_native.sh "$NATIVE_WS" -j2
bash tools/check_native.sh "$NATIVE_WS"
```

一个 Catkin 源空间选择现有目录中的 14 个包，同时生成两种模式的参数。一份 `slicer_air_ground_msgs` 供 V1 和 UAV 客户端复用。开发与稳定运行分别使用独立源码检出和绝对路径不同的工作区。每个终端 source 对应工作区的 `install/setup.bash`。

## 包和运行入口

| 职责 | 位置 / 原生入口 |
|---|---|
| 传感器、LIO、局部 EKF | `src/Online_Navigation_Without_RTK/catkin_ws/src` 中的 `slicer_bringup`、`fast_lio`、`livox_ros_driver2`。局部 EKF 发布 `/odom` 和 `odom → base_link`。 |
| 无 RTK 导航 | `slicer_navigation_without_mapping/slicer_nav_without_mapping.launch`。Gmapping 管 `map → odom`，B 样条 + TEB，Action `/move_base`。 |
| RTK 导航 | `ugv_bringup` 用 `slicer_field_sensors.launch`、`slicer_body_driver.launch` 和 `slicer_field_navigation.launch`。DWA、Action `/ugv/move_base`。配准后由全局 EKF 拥有 `map → odom`。 |
| 车体控制 | 两种模式走各自的 Guard 和 Mux。导航输出到 `/cmd_vel/nav_raw`，Guard 发 `/cmd_vel/nav`，当前模式的 Mux 独占 `/cmd_vel`，driver 独占底盘串口。 |
| 空地任务 | V1 是 `ugv_bringup/slicer_air_ground.launch`。V2 是 `ag_ugv_task/ugv.launch`，源码在 `ag_v2_ws/src/AG_Client`，基于旧盘上游提交 `a2003d4`。 |
| 通信、现场工具 | `src/conmunication` 提供 `ugv_communication` 与 V1 的 FKIE 接入，V2 有自己的白名单。机电、固件和测试工具见 [Hardware](Hardware/README.md) 和 [Test](Test/README.md)。 |

**无 RTK 模式**使用生成的参数目录和整车硬件路径。在候选仓根目录启动，按现场情况决定是否打开 GUI。

```bash
source "$NATIVE_WS/install/setup.bash"
roslaunch slicer_navigation_without_mapping slicer_nav_without_mapping.launch \
  runtime_root:="$NATIVE_WS/runtime/no_rtk" workspace2_root:="$PWD" gui:=false
```

**RTK 模式**分别起定位、底盘和导航，顺序参照[户外操作单](src/Online_Navigation_with_RTK/docs/field/AIR_GROUND_FIELD_RUNBOOK.md)。该操作单里的 `run.sh` 和旧 IP 对应原容器环境。原生定位入口所需的 GNSS YAML 由操作员准备，按[依赖清单](docs/DEPENDENCIES.md)填实际串口、差分设置和外部凭据路径。

```bash
# 各终端 source 同一 install，使用同一车端 ROS master
roslaunch ugv_bringup slicer_field_sensors.launch config_file:="$HOME/.config/slicer/gnss.yaml"
roslaunch ugv_bringup slicer_body_driver.launch
roslaunch ugv_bringup slicer_field_navigation.launch geographic:=false
# 配准成功后仅替换上面的导航终端，定位和底盘保持
roslaunch ugv_bringup slicer_field_navigation.launch
```

**空地执行器只选一个。** 当前原生整合以 V1 为执行基线，依据是车端 V1 运行文件散列与已验证提交相符。11 月 19 日最新 V2 记录证实执行模式进程与话题连接。V2 先按下方命令仅观测，确认飞机选用 V2 且完成验证后再由现场负责人切换为执行。直接 V2 launch 默认 `monitor_only=false`，旧 Compose 默认 `true`，旧车端 `bridge.env` 明确覆盖为 `false`。启动时显式写明模式和 FKIE 归属。

```bash
# V1
roslaunch ugv_bringup slicer_air_ground.launch
# 或 V2 静态观测，沿用已有同步或同一 master
roslaunch ag_ugv_task ugv.launch monitor_only:=true start_fkie:=false
```

切导航模式时先结束原任务和原模式控制链，确认只有一套 `map → odom`、最终速度发布者和串口所有者。隔离 Focal amd64 已完成 Catkin 构建与安装检查，原生运行和设备验证见 [HANDOFF.md](HANDOFF.md)。
