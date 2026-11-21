# 整车操作手册

从仓库根目录执行。先完成 [构建](../README.md#从克隆到构建) 和 [设备配置](DEPENDENCIES.md#设备和现场配置)。需要跨机通信时，为每台机器设置其 master 和可回连的本机 `ROS_IP`。ROS 终端应使用同一整车安装版本。

## 无 RTK 导航

```bash
export SLICER_NATIVE_WS="$HOME/slicer_noetic_current"  # 或指定你的完整安装工作区
./tools/no_rtk.sh nav --background
```

本入口使用所选工作区生成的 `runtime/no_rtk`，源配置位于 [no_rtk](../src/ugv_bringup/config/no_rtk/)。未设置 `SLICER_NATIVE_WS` 时优先使用车端的 `$HOME/slicer_noetic_current` 安装。启动前核对 launch 节点和 MID360 配置中的主机地址。当前车端 MID360 通过 Jetson `eth0` 连接，主机 `192.168.1.5/24`，雷达 `192.168.1.192`。检查 `ip -br -4 addr show eth0` 和 `rostopic hz /livox/lidar`。

启动脚本使用同一用户已登录的 Jetson 本地 X11 桌面。未登录桌面时以无界面模式运行，也可传入 `gui:=false`。导航已运行时可用 `./tools/no_rtk.sh rviz` 单独打开本地 RViz、目标坐标窗口和录点节点。

真实底盘启动前会检查 `Hardware/config.yaml` 中显式配置的串口路径。若当前终端早于账号加入 `dialout`，脚本会为本次启动加载账号已有的组权限。串口不可访问时会在启动前显示设备路径和原因。

- **2D Nav Goal** 在地图上点住并拖动确定朝向，经 `/move_base_simple/goal` 发送给 `/move_base`。
- **Record Waypoint** 可连续点击打点。录点节点写入 `src/slicer_navigation_without_mapping/config/waypoints.yaml`，可用 `rostopic echo /slicer/waypoint_record/status` 确认 `SAVED`。巡航读取该文件。在线 SLAM 重新建图后需要重新确认航点坐标。

Gmapping 发布 `map → odom`，局部 EKF 发布 `odom → base_link`。用 `./tools/no_rtk.sh stop` 结束登记的后台导航。单独运行的 `rviz` 入口可在启动终端按 Ctrl+C 结束。

## RTK 导航与空地接口

各终端先执行 `source "$SLICER_NATIVE_WS/install/setup.bash"`。GNSS 配置按依赖清单建立；RTK 的站点、杆臂和规划配置在 [ugv_bringup/config](../src/ugv_bringup/config/)。

```bash
# 定位终端
roslaunch ugv_bringup slicer_field_sensors.launch config_file:="$HOME/.config/slicer/gnss.yaml"
# 底盘终端
roslaunch ugv_bringup slicer_body_driver.launch
# 起步导航终端
roslaunch ugv_bringup slicer_field_navigation.launch geographic:=false
# 已配准且停车后，用下行替换起步导航 launch，定位保持运行
roslaunch ugv_bringup slicer_field_navigation.launch
```

状态查看用 `rosrun cup_gnss gnss_status.py`、`rostopic echo -n 1 /geographic/status`。配准、地理目标与停止步骤见 [RTK 现场手册](rtk/field/AIR_GROUND_FIELD_RUNBOOK.md)。RTK 全局 EKF 发布 `map → odom`，局部 EKF 继续负责 `odom → base_link`，DWA 的 Action 为 `/ugv/move_base`。

```bash
# 选择一个执行器。V1
roslaunch ugv_bringup slicer_air_ground.launch
# 或选择 V2 仅观测。已有对应 FKIE 或同一 master 时
roslaunch ag_ugv_task ugv.launch monitor_only:=true start_fkie:=false
```

V2 用独立 FKIE 时显式设置 `start_fkie:=true uav_ip:="$UAV_IP"`。V2 真正执行使用 `monitor_only:=false`。V1 的双 master 入口见[通信包说明](../src/ugv_communication/README.md)。切换模式或执行器前结束旧任务、保持停车并停止旧入口。ROS 控制在各模式经 Guard 和 Mux，单一 Mux 发布 `/cmd_vel`，单一 driver 占用底盘串口。

## 底盘通信锁存恢复与测试

使用 C30D v1.7 固件和本节配套 ROS 安装。无 RTK 和 RTK 都按正常入口启动。12 V 接通且 Motor ON 时，驱动检测到 MCU 的 UART、速度指令看门狗或测试锁存后保持零输出，并在固件确认四轮静止、零 PWM 持续 250 ms 后自动运行 `unlock.py --unlock`。驱动确认清锁后丢弃故障期间的速度指令，等待新的 `/cmd_vel`。编码器异常、堵转、转向错误或控制周期异常仍按故障停车。

尚未安装 v1.7 的板先停车并停止占用底盘的程序。确认 Motor OFF、物理断开 12 V、仅 USB 供电后，烧录随仓库交付的镜像。烧录校验通过后先在断动力状态检查零输出，再接回 12 V。

```bash
python3 -s Hardware/Flashing/flash_v1_7_verified.py --power-off-confirmed
python3 -s Hardware/zero_check.py
```

两个模式的操作终端使用同一个 v1.7 ROS 安装。

```bash
export SLICER_NATIVE_WS="$HOME/slicer_noetic_c30d_v17"
source "$SLICER_NATIVE_WS/install/setup.bash"
```

新检出没有该 ROS 安装时，先按 [构建说明](../README.md#从克隆到构建) 用 `bash tools/build_native.sh "$HOME/slicer_noetic_c30d_v17" -j2` 构建。旧终端缺少已授予的 `dialout` 组时，烧录和 RTK 底盘驱动可以从新登录的终端启动，或先执行 `sg dialout`。

短时遥测中断时，驱动保持停止并等待新遥测，最多等到距最后一帧 3 秒。设备被拔掉或串口返回 I/O 错误时，按原入口结束后重启。测试锁存模拟固件锁存和恢复，不断开 USB。

在 Jetson 上从仓库根目录运行这两个命令。导航运行时，命令把操作交给当前底盘驱动。挂机且串口空闲时，命令直接连接底盘。

```bash
python3 -s Hardware/LockDown/lock.py
python3 -s Hardware/Unlock/unlock.py --unlock
```

首次带动力测试请架空车轮。测试锁定需要 12 V 接通、Motor ON、四轮静止且 PWM 为零。`[LOCKED]` 表示已收到下位机测试锁存遥测。挂机时锁存会保留，手动执行解锁命令可清除。两种导航运行期间，驱动会自动调用解锁工具并记录 `[UNLOCKED]` 和 `MCU_RECOVERED`。

可用 `rostopic echo /slicer_c30d_driver/ready` 观察导航期间的就绪状态。独立 `unlock.py --check` 在串口空闲时读取故障遥测。安装和回退见 [C30D v1.7 固件说明](../Hardware/Firmware/v1.7/README.md)。

## 独立硬件工具

这些工具在仓库根目录运行。使用直接设备入口前，先停止占用同一设备的程序。底盘测试锁定和解锁也可通过运行中的导航驱动执行。按键、极性、限幅、设备匹配见[遥控说明](../tools/manual/README.md)和 [Hardware/config.yaml](../Hardware/config.yaml)。

| 功能 | 旧入口 | 新入口 |
|---|---|---|
| 键盘 | `./Test/run_wasd.sh` | `./tools/manual/run_wasd.sh` |
| 手柄 | `./Test/run_gamepad.sh` | `./tools/manual/run_gamepad.sh` |
| 检查解锁 | `python3 -s Hardware/Unlock/unlock.py --check` | `python3 -s Hardware/Unlock/unlock.py --check` |
| 解锁底盘 | `python3 -s Hardware/Unlock/unlock.py --unlock` | `python3 -s Hardware/Unlock/unlock.py --unlock` |
| 触发底盘测试锁存 | `rosservice call /slicer_c30d_driver/test_chassis_lock '{}'` | `python3 -s Hardware/LockDown/lock.py` |
| 无 RTK 启动 | 在旧 NoRTK 目录运行 `./run.sh nav --background` | `./tools/no_rtk.sh nav --background` |
| 无 RTK 停止 | 同目录 `./run.sh stop` | `./tools/no_rtk.sh stop` |
| B15 舵机 | `./Test/run_b15_servo.sh --run` | `./tools/manual/run_b15_servo.sh --run` |
| 气夹 | `./Test/run_airgrip.sh --run` | `./tools/manual/run_airgrip.sh --run` |
| 组合手柄 | `./Test/tests/run_combined_gamepad.sh --run` | `./tools/manual/run_combined_gamepad.sh --run` |
| PWM 舵机 | `./Test/run_pwm_servos.sh` | `./tools/manual/run_pwm_servos.sh` |
| 正式视觉伺服 | `./Test/Visual_Grasp/run.sh` | source 整车 install 后执行 `rosrun slicer_visual_grasp run.sh` |

组合手柄的配置、按键和协议测试已随原实现恢复。视觉界面、配置和无设备检查见 [slicer_visual_grasp](../src/slicer_visual_grasp/README.md)。`Vision_adjust` 只作为[历史参考](archive/VISION_ADJUST.md)保留在提交历史。

舵机和组合手柄独占 C30D，运行前先停止正在占用底盘串口的导航或其他工具。气夹单独运行时也需要独占其串口。三个入口会复用账号已有的 `dialout` 权限。停止工具后换插 USB 口，再用原命令启动。C30D 通过 `/dev/c30d` 定位，AirGrip 和组合手柄通过 USB by-id 路径定位。
