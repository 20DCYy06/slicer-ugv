# 整车操作手册

先完成[构建](../README.md#从克隆到构建)和[设备配置](DEPENDENCIES.md#设备和现场配置)。跨机通信需要在各机设置 ROS master 和可回连的本机地址。

## 选择发布和源码版本

正式启动从 `~/workspace/slicer/run` 进入。该入口解析 `current` 指向的发布，并使用同一发布的源码脚本、安装、生成配置和固定源码依赖。

```bash
BASE="$HOME/workspace/slicer"
"$BASE/run" status
"$BASE/run" rtk
# 或按现场约定选择无 RTK
"$BASE/run" no-rtk nav --background
```

`run shell` 打开该发布的环境，也可执行单个命令，例如 `"$BASE/run" shell rospack find ugv_navigation`。开发使用对应工作区的 `src/slicer-ugv` 和 `install/setup.bash`，并在同一工作区运行下面的 `tools/` 入口。

发布的 LiDAR/规划/控制默认值来自确定的源码与生成配置。现场 GNSS 和差分凭据、相机标定沿用外部配置，单独记录生效位置与修订。录点、ROS 会话、日志和采集保存在发布外的 `site/state/<release-id>`、`logs/<release-id>`、`data/<release-id>`。

切换时先在旧入口结束任务、停稳、关闭导航与底盘进程，记录旧 `current`。按[维护说明](MAINTENANCE.md#候选发布与切换)验收新发布、切换链接、核对包路径，再按当前模式启动。失败时停止新入口并恢复链接。固件、现场配置和标定分别按各自记录恢复。

## 无 RTK 导航

```bash
./tools/no_rtk.sh nav --background
```

开发入口使用同一工作区的 `install` 和 `runtime/no_rtk`。源配置位于 [no_rtk](../src/ugv_bringup/config/no_rtk/)。启动前按现场网络和雷达配置核对主机地址，用 `ip -br -4 addr` 和 `rostopic hz /livox/lidar` 验证。

启动脚本使用同一用户已登录的 Jetson 本地 X11 桌面。未登录桌面时以无界面模式运行，也可传入 `gui:=false`。导航已运行时可用 `./tools/no_rtk.sh rviz` 单独打开本地 RViz、目标坐标窗口和录点节点。

真实底盘启动前会检查生成的 `runtime/no_rtk/config/c30d_host.yaml` 中的串口路径，生成输入取显式 `SLICER_HARDWARE_CONFIG`，默认取固定硬件仓配置。正式入口固定所选发布的生成配置与录点路径。若当前终端早于账号加入 `dialout`，脚本会为本次启动加载账号已有的组权限。串口不可访问时会在启动前显示设备路径和原因。

无 RTK 导航的前进上限为 0.8 m/s，倒车上限为 0.5 m/s，线加速度限制为 0.40 m/s²。TEB 向前规划 3.0 m，局部代价地图为 7×7 m。已生成的 `runtime/no_rtk` 需要更新后重启导航。

- **2D Nav Goal** 在地图上点住并拖动确定朝向，经 `/move_base_simple/goal` 发送给 `/move_base`。
- **Record Waypoint** 可连续点击打点。录点节点写入所选运行状态目录的 `waypoints.yaml`，可用 `rostopic echo /slicer/waypoint_record/status` 确认 `SAVED`。巡航读取该文件。在线 SLAM 重新建图后需要重新确认航点坐标。

Gmapping 发布 `map → odom`，局部 EKF 发布 `odom → base_link`。用 `./tools/no_rtk.sh stop` 结束登记的后台导航。单独运行的 `rviz` 入口可在启动终端按 Ctrl+C 结束。

## RTK 导航与空地接口

各终端使用同一个发布或开发安装。GNSS 配置按依赖清单建立，RTK 参数在 [ugv_bringup/config](../src/ugv_bringup/config/)。

```bash
# 各终端先进入发布环境，开发时 source 对应工作区的 install/setup.bash
"$HOME/workspace/slicer/run" shell

# 定位终端
roslaunch ugv_bringup slicer_field_sensors.launch config_file:="$HOME/.config/slicer/gnss.yaml"
# 底盘终端
roslaunch ugv_bringup slicer_body_driver.launch
# 导航终端
roslaunch ugv_bringup slicer_navigation_runtime.launch
```

首次合法 WGS84 目标由共享运行器自动准备并接续，无需飞机重发。它使用现有 odom 规划、Guard/Mux 和 RTK 轨迹拟合，停稳后等待新鲜全局输出，再切换到 `map`。可由飞机发送一次任务，或用 `rosrun ugv_navigation geographic.py send --target-file "$MARK"` 提交。预览 `geographic.py plan` 只查询，未配准时报告原因。可观察 `/geographic/navigation_runtime` 和 `/geographic/status`。

状态查看用 `rosrun cup_gnss gnss_status.py`、`rostopic echo -n 1 /geographic/status`。配准、地理目标与停止步骤见 [RTK 现场手册](rtk/field/AIR_GROUND_FIELD_RUNBOOK.md)。RTK 全局 EKF 发布 `map → odom`，局部 EKF 继续负责 `odom → base_link`，DWA 的 Action 为 `/ugv/move_base`。

```bash
# 选择一个执行器。V1
roslaunch ugv_bringup slicer_air_ground.launch
# 或选择 V2 仅观测。已有对应 FKIE 或同一 master 时
roslaunch ag_ugv_task ugv.launch monitor_only:=true start_fkie:=false
```

V2 用独立 FKIE 时显式设置 `start_fkie:=true uav_ip:="$UAV_IP"`。V2 真正执行使用 `monitor_only:=false`。V1 的双 master 入口见[通信包说明](../src/ugv_communication/README.md)。切换模式或执行器前结束旧任务、保持停车并停止旧入口。ROS 控制在各模式经 Guard 和 Mux，单一 Mux 发布 `/cmd_vel`，单一 driver 占用底盘串口。

### RTK 本地分屏桌面

Jetson 登录本地 X11 桌面后，从仓库根目录执行。RViz 和状态栏在同一个窗口内，状态栏固定在右侧。脚本使用 `ROS_MASTER_URI`，本机 Master 不存在时会启动它。终端使用同一个 Master 即可查看和控制节点。

```bash
./tools/rtk_desktop_ui.sh
```

看板用固定的单行位置显示 RTK 解算、卫星数、配准、MID360 扫描、LIO、底盘、Mux、当前位置和当前目标。普通状态持续约 0.45 秒后更新，急停和保持停车会立即显示。目标校验使用原始实时状态。RViz 固定坐标系随运行中的全局代价地图切换，启动阶段默认显示 `odom`。

RViz 绘制上限为 15 FPS，右栏每 250 毫秒检查显示变化。启动前的本机进程检查在后台进行，超过 2 秒会提示重试。配准确认为独立小窗。点击 **一键全部停止** 会取消待确认或正在检查的步骤。

黑白占据栅格中白色为空闲、黑色为障碍、灰色为未知，实时扫描和膨胀代价仍叠加显示。有已对齐 RTK 坐标系的 ROS 地图文件时，可在启动前设置 `SLICER_RTK_MAP_YAML=/绝对路径/map.yaml`，默认坐标系为 `map`，可通过 `SLICER_RTK_MAP_FRAME` 指定实际坐标系。没有指定文件时优先显示 `/map`，其余情况由 RTK 当前全局代价地图生成黑白栅格。

共享 RTK 导航的准备配置限速 0.25 m/s，切换后的 `map` 导航上限为 0.8 m/s、45°/s（0.785398 rad/s），线加速度限制为 0.40 m/s²。局部代价地图为 9×9 m，供 Guard 检查全速制动扫掠范围。两种导航的保护距离均继续按 0.20 m/s² 制动减速度计算。现有 C30D 固件的 PWM 使用整数档，900‰ 上限对应最高可用档 1429/1600 ticks（89.31% 占空比）。重启导航和底盘驱动后使用新参数。DWA 参数在 `src/ugv_bringup/config/dwa_field.yaml`，PWM 档位在 `src/ugv_bringup/config/slicer_driver_body_speed.yaml`。

- 打开桌面后，先点击 **1. 启动定位与传感器 → 2. 启动底盘驱动**。底盘收到就绪状态后显示 **已完成**，可选择第 3 步抓取或直接启动 **4. 启动导航**。黄色按钮会显示当前等待的状态。定位默认读取 `~/.config/slicer/gnss.yaml`，可通过 `SLICER_GNSS_CONFIG` 选择外部 YAML。
- **3. 移动并抓取** 通过 ROS 启动已安装的 `slicer_visual_grasp`，调用 `--run` 的正式抓取动作。后台相机由桌面按钮操作。车辆停稳后，面板暂停自己启动的底盘驱动，等待串口释放再执行抓取。收到抓取和抬升完成的结果后，按钮变绿并显示 **3. 释放**，底盘驱动自动恢复。保持抓取即可继续第 4 步和后续流程。需要释放时再次点击第 3 步，释放和回升完成后按钮变红。导航已经运行时，等车辆停稳且导航和空地任务空闲即可操作抓取或释放。导航节点继续运行，面板临时保持停车，完成后只解除本次操作建立的停车。外部终端启动的底盘驱动由原终端管理。
- **导航诊断** 提供手动直行配准与 `map` 切换。常规任务由共享导航自动准备。可在桌面 **发送 WGS84 目标**，或启动一个空地执行器等待飞机任务。准备时同时显示真实终点和阶段原因。
- **飞机通信节点** 在同一行选择 V1 或 V2，然后点击按钮启动。黄色「车端已启动 · 等飞机」表示等待对端消息。绿色表示近期收到 V1 的 `/uav/test`、V1 当前车端会话的命令或 V2 的 `/air_ground/v2/uav_state`。同步网桥或共享 Master 也可承载这些消息。独立 FKIE 链路可在启动桌面前设置 `SLICER_UAV_IP`。
- 已从其他终端启动的步骤同样显示实际就绪状态。点击该步骤会提示其来源，需要重启时在原终端停止。完整的子进程日志保存在 `ROS_LOG_DIR`（开发时默认位于 `W/logs/ros`）。
- **2D Nav Goal** 在 RViz 平面按住并拖动指定位置和朝向。同一桌面发送的新目标立即交给 `move_base` 替换上一目标。「正在前往」一行显示最新目标的 WGS84 纬度、经度。配准或 TF 尚不可用时显示等待提示。Mux 保持停车、急停、配准采集或其他控制端占用导航时，右侧显示当前阻塞原因。
- **Waypoint 打点** 可在地图地面平面连续点击。启动终端打印 WGS84 纬度、经度，坐标转换未就绪时显示等待提示。
- **一键全部停止** 依次关闭面板启动的进程，并反馈停止状态。点击工具栏的 **关闭** 或按 **Esc** 也会清理这些进程。可用 `SLICER_NATIVE_WS` 选择与现有导航一致的 ROS 安装目录。

## 底盘通信锁存恢复与测试

使用 C30D v1.7 固件和本节配套 ROS 安装。无 RTK 和 RTK 都按正常入口启动。12 V 接通且 Motor ON 时，驱动检测到 MCU 的 UART、速度指令看门狗或测试锁存后保持零输出，并在固件确认四轮静止、零 PWM 持续 250 ms 后自动运行 `unlock.py --unlock`。驱动确认清锁后丢弃故障期间的速度指令，等待新的 `/cmd_vel`。编码器异常、堵转、转向错误或控制周期异常仍按故障停车。

固件检查和升级在所选发布的 `run shell` 环境执行。尚未安装 v1.7 的板先停车并停止占用底盘的程序。确认 Motor OFF、物理断开 12 V、仅 USB 供电后，烧录随仓库交付的镜像。烧录校验通过后先在断动力状态检查零输出，再接回 12 V。

```bash
python3 -s "$SLICER_HARDWARE_ROOT"/Flashing/flash_v1_7_verified.py --power-off-confirmed
python3 -s "$SLICER_HARDWARE_ROOT"/zero_check.py
```

两个模式的操作终端使用同一个 v1.7 ROS 安装。

```bash
"$HOME/workspace/slicer/run" shell
```

开发时按[构建说明](../README.md#从克隆到构建)构建对应工作区。旧终端缺少已授予的 `dialout` 组时，烧录和 RTK 底盘驱动可以从新登录的终端启动，或先执行 `sg dialout`。

短时遥测中断时，驱动保持停止并等待新遥测，最多等到距最后一帧 3 秒。设备被拔掉或串口返回 I/O 错误时，按原入口结束后重启。测试锁存模拟固件锁存和恢复，不断开 USB。

在 Jetson 上从仓库根目录运行这两个命令。导航运行时，命令把操作交给当前底盘驱动。挂机且串口空闲时，命令直接连接底盘。

```bash
python3 -s "$SLICER_HARDWARE_ROOT"/LockDown/lock.py
python3 -s "$SLICER_HARDWARE_ROOT"/Unlock/unlock.py --unlock
```

首次带动力测试请架空车轮。测试锁定需要 12 V 接通、Motor ON、四轮静止且 PWM 为零。`[LOCKED]` 表示已收到下位机测试锁存遥测。挂机时锁存会保留，手动执行解锁命令可清除。两种导航运行期间，驱动会自动调用解锁工具并记录 `[UNLOCKED]` 和 `MCU_RECOVERED`。

可用 `rostopic echo /slicer_c30d_driver/ready` 观察导航期间的就绪状态。独立 `unlock.py --check` 在串口空闲时读取故障遥测。安装和回退见 [硬件仓的固件说明](https://github.com/20DCYy06/slicer_ugv_hardware)。

## 独立硬件工具

这些工具在仓库根目录运行。使用直接设备入口前，先停止占用同一设备的程序。底盘测试锁定和解锁也可通过运行中的导航驱动执行。按键、极性、限幅、设备匹配见[遥控说明](../tools/manual/README.md)和 清单所选硬件仓的 `config.yaml`。

| 功能 | 旧入口 | 新入口 |
|---|---|---|
| 键盘 | `./Test/run_wasd.sh` | `./tools/manual/run_wasd.sh` |
| 手柄 | `./Test/run_gamepad.sh` | `./tools/manual/run_gamepad.sh` |
| 检查解锁 | `python3 -s "$SLICER_HARDWARE_ROOT"/Unlock/unlock.py --check` | `python3 -s "$SLICER_HARDWARE_ROOT"/Unlock/unlock.py --check` |
| 解锁底盘 | `python3 -s "$SLICER_HARDWARE_ROOT"/Unlock/unlock.py --unlock` | `python3 -s "$SLICER_HARDWARE_ROOT"/Unlock/unlock.py --unlock` |
| 触发底盘测试锁存 | `rosservice call /slicer_c30d_driver/test_chassis_lock '{}'` | `python3 -s "$SLICER_HARDWARE_ROOT"/LockDown/lock.py` |
| 无 RTK 启动 | 在旧 NoRTK 目录运行 `./run.sh nav --background` | `./tools/no_rtk.sh nav --background` |
| 无 RTK 停止 | 同目录 `./run.sh stop` | `./tools/no_rtk.sh stop` |
| B15 舵机 | `./Test/run_b15_servo.sh --run` | `./tools/manual/run_b15_servo.sh --run` |
| 气夹 | `./Test/run_airgrip.sh --run` | `./tools/manual/run_airgrip.sh --run` |
| 组合手柄 | `./Test/tests/run_combined_gamepad.sh --run` | `./tools/manual/run_combined_gamepad.sh --run` |
| PWM 舵机 | `./Test/run_pwm_servos.sh` | `./tools/manual/run_pwm_servos.sh` |
| 正式视觉伺服 | `./Test/Visual_Grasp/run.sh` | source 整车 install 后执行 `rosrun slicer_visual_grasp run.sh` |

组合手柄的配置、按键和协议测试已随原实现恢复。视觉界面、配置和无设备检查见 [slicer_visual_grasp](../src/slicer_visual_grasp/README.md)。`Vision_adjust` 只作为[历史参考](archive/VISION_ADJUST.md)保留在提交历史。

舵机和组合手柄独占 C30D，运行前先停止正在占用底盘串口的导航或其他工具。气夹单独运行时也需要独占其串口。三个入口会复用账号已有的 `dialout` 权限。停止工具后换插 USB 口，再用原命令启动。C30D 通过 `/dev/c30d` 定位，AirGrip 和组合手柄通过 USB by-id 路径定位。
