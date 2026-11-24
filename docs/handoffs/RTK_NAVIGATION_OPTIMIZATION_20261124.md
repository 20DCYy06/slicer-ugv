# RTK 导航优化

任务分支为 `RTK导航优化`，从当时的 `origin/main`（`a7a64c70698d07779b1be77ae7059fcc877245b9`）创建。工作树是 `~/workspace/slicer/dev/rtk-navigation-optimization/src/slicer-ugv`。现场诊断依据是 2026-11-23 速度排查交接（`dev/rtk-field-speed-audit/src/slicer-ugv/docs/handoffs/RTK_FIELD_SPEED_AUDIT_20261123.md`）和 `~/workspace/slicer/data/rtk-corner-20261123-084654/analysis/speed-chain/`。硬件依赖为 `a377b45c48c4be91add25b73eacedabdb8c264de`，固件改动为零。任务验证使用独立安装和回环 ROS master。正式 `current` 仍是 `20261123-rtk-fixed-a7a64c7`。

## 变更

- RTK 全局、局部膨胀半径先由 0.24 m 增至 0.48 m，随后按现场要求设为 0.25 m。两张地图均在 4.5 m 内标障和清除，使全局规划与局部控制尽早使用同一批新障碍。车身按现场给出的 22 cm 计，现有软件横向足迹宽 28 cm，名义上每侧多 3 cm。膨胀距离从障碍格向外计，扩大高代价区域。全局搜索、路线复用与平滑共用已观察障碍的代价门槛，约相当于路径中心离障碍格 0.23 m，分辨率会影响落在哪一格。Guard 的硬足迹维持原尺寸。旧回放最近点云距软件足迹约 5 cm，不能据此推定所有路况的最小物理净空。要提高强制最小净空，需单独量车体外缘及传感器误差并复测直角和窄道。
- RTK 复用 NoRTK 的 Dijkstra 与 B 样条规划器，使用连续梯度回溯以更早转向，随着扫描推进分段平滑路径的已观察前缀，其余段保留临时规划，Action 终点仍是原目标。已有路线仍有效且代价相近时继续走该路线，新增障碍阻断时重规划。RViz `Nav Goal` 允许点击全局网格内的未知区，局部 DWA 和 Guard 按已扫清的可行空间行驶。缺少可通行的局部路线时停车，目标超出全局地图网格时无法规划。配准选择的候选点和路线仍须通过已观察区域检查。
- 正式导航上限保留 0.8 m/s 和 45°/s。DWA 将前进速度采样从 3 个增加到 7 个，并提高跟踪全局绕障路线的权重。Guard 在原指令过快、实际 LIO 位移及上一帧指令均可在已观察范围内制动时，沿现有前进弧线寻找较低可用速度，并按每个候选速度单独校验制动包络。制动净空不足、输入过期、规划无可用前进指令时停车。底盘和固件参数不变。
- 配准 DWA 的线速上限由 0.25 m/s 增至 0.35 m/s，角速度、加速度、质量门和停车确认沿用原值。拟合增量处理新配对，样本淘汰或重置时重建缓存。

## 桌面入口、起步与 Waypoint

1. 「启动 RTK 与配准」启动 GNSS 与全局定位并安排往返。「启动雷达与底盘」启动 MID360、本地定位与底盘驱动。「启动导航」使用共享运行器，普通 2D Nav Goal 和 RTK WGS84 使用同一导航链。飞机通信和一键全部停止保留。抓取依然有独立命令入口。
2. 只按后两步启动，或暂时收不到 Fixed 时，普通 `odom` 导航可以按 0.8 m/s 上限运行。RTK 已安排且条件满足后，在任务空闲、停车和 Mux 就绪时切到 0.35 m/s 的配准档，再发起一次前进 2 m、停车拟合、沿原路低速倒回。返程倒车上限 0.15 m/s，到距起点 0.12 m 容差内并停稳后切到 `map`。已有有效配准直接申请切换。返程失败停止并显示原因。
3. 普通 `odom` 和正式 `map` 目标都使用同一闭环对向。先把可行路线转换、裁剪到 DWA 的局部跟踪窗口，取开头约 0.20 m 的短弦近似起点切线，记录当前 TF 朝向、切线和差值。PD 使用 TF 航向误差和实测角速度，(`Kp=1.6`、`Kd=0.25`)。转速上限 0.45 rad/s，误差容差 0.12 rad，转动后停稳 0.25 s 再交回 DWA。16 s 未完成时报告失败。0.20 m 内的短目标由 DWA 到位控制处理。Guard 和 Mux 全程维持原控制权限。
4. 选「Waypoint 打点」，在 RViz 点地面，启动终端立即打印 WGS84 纬度和经度，桌面询问是否导航。确认走共享 WGS84 入口，取消仅保留坐标。具备可用地理变换即可打点，现场无需另录参考点。

## 验证

- 隔离 Catkin 构建、安装了 `slicer_navigation`、`ugv_bringup`、`ugv_navigation`、`ugv_slicer_control`。底层已安装依赖用于测试，生成配置保存于任务的临时构建目录。离线 332 项、C++ 路径检查和 7 项针对性的 ROS launch 参数检查通过。旧的自定义巡航档参数测试在基线也因加速度覆盖预期不一致而失败。详细记录见 `/tmp/rtk-navigation-optimization-build.jfiDky8r/`。
- Guard 的隔离 ROS 传输测试 12 项通过。合成场景确认 0.8 m/s 上限、有制动余地时自动降速，以及不满足制动或定位条件时停车。证据在 `/tmp/rtk-guard-ros-legacy/` 和隔离构建目录的 `guard-final.log`。
- 配准 CLI 模拟目标经过起步、自动配准、地图切换并到点。合成 180 个配对和 90 次拟合在该 Jetson 上的处理中位耗时从 126 ms 降到 32 ms，实际总时间仍受雷达、Fixed、停车和切换影响。证据见隔离构建目录的 `final-registration-cli.log`。
- 离屏 Qt 和隔离 ROS 验证配准前录两点、打点坐标复制及里程计失效，结果见 `/tmp/slicer-waypoint-edit/waypoint-loopback-result.log`。
- 隔离仿真使用真实 `move_base`、Guard 和 Mux。开阔场景在 10.21 s 到达起初未知的 6 m 目标，Mux 峰值为 0.800 m/s。遮挡墙场景发现墙后改道，在横向约 0.98 m 处越过墙所在截面，37.51 s 到达原 6 m 目标。网格折线的对照在墙前爬行，65 s 内未到点。证据见隔离构建目录的 `final-unknown-goal.log` 和 `final-detour.log`。
- 实车最高速度、最小净空与窄道通过性、配准总耗时、RViz 实际鼠标操作与打点地理误差待现场验证。

## 11 月 24 日现场复查与坐标读数

- 桌面将两行位置合为「当前位置」，可右键复制 WGS84 经纬度。配准前使用新鲜 Fixed 天线坐标，具备可用配准定位时优先使用车体参考点。没有新鲜坐标时显示「—」。Waypoint 和 WGS84 任务使用各自的质量检查与数据源。332 项离线测试通过。隔离 ROS 与离屏 Qt 用合成 Fixed 验证了单行显示、右键复制、过期清空和已有 Waypoint 回归，日志见 `/tmp/slicer-rtk-coordinate-readout/current-position-ros-qt.log`。
- 05:25–05:26 的最新现场会话仍从正式发布 `a7a64c7` 启动。RViz 的两次目标均在 `odom` 坐标系，DWA 的 `max_vel_x` 和 Guard 启动限速均为 0.25 m/s。该会话只启动了起步导航配置，处于正式 `map` 导航前。较早的 04:37 `map` 日志确认正式档为 0.8 m/s。日志位于 `~/workspace/slicer/logs/20261123-rtk-fixed-a7a64c7/ros/`，其中最新导航日志是 `rtk-navigation-odom-zv86ynmz.log`，对应 ROS 会话是 `4e96b1d8-e80a-11f1-a504-3c6d661ed63d`。
- 当日最新会话没有同步速度与 MCU 遥测录包，无法回算该趟实车速度。已有 11 月 23 日 08:15 正式 `map` 录包中，DWA、Mux 与 MCU 目标的线速峰值均为 0.404 m/s，PWM 峰值 903 ticks，低于会话上限 1429 ticks。这一趟的约 0.4 m/s 在规划器指令中已出现。后一份录包还记录了拐角处 DWA 局部规划失败、长时间偏向转向的情况。同一 08:15 录包编码器反馈峰值 0.425 m/s，LIO 位姿差分峰值 0.227 m/s。需用地面距离区分反馈尺度与滑移，现有证据不足以归因于 PID。后续实车记录应同步保存 `/cmd_vel/nav_raw`、`/cmd_vel/nav`、`/cmd_vel`、底盘诊断、`/wheel/odom` 和 `/lio/odom`，并记录运行档位。

## 独立候选安装

2026-11-24 在 `~/workspace/slicer/dev/rtk-navigation-optimization` 从程序提交 `7f0bfdb102348e42e30f1cd6b8d8a01138fee2f8` 完整构建 Livox SDK 和 Catkin 安装。四项依赖分别检出 `dependencies.repos` 固定版本。该工作区有独立的构建、安装、运行配置和验证日志。原生安装检查通过，包路径均解析到候选 `install`，正式档和配准档分别解析为 0.8 m/s 和 0.35 m/s，两张代价图的膨胀半径均为 0.48 m。用候选安装的回环 ROS 与离屏 Qt 复测了「当前位置」单行、复制、过期清空和 Waypoint。证据见任务工作区 `logs/build-7f0bfdb.log`、`logs/check-7f0bfdb.log` 和 `logs/candidate-7f0bfdb.log`。程序尚待现场试车。

在 Jetson 本地 X11 新终端启动候选，切换前由现场执行会话停稳并退出原控制链。

```bash
W="$HOME/workspace/slicer/dev/rtk-navigation-optimization"
cd "$W/src/slicer-ugv"
SLICER_NATIVE_WS="$W" ./tools/rtk_desktop_ui.sh
```

正式日常入口 `~/workspace/slicer/run rtk` 由 `current` 选择发布。本次 `./run status` 核对到 `a7a64c7`。测试版在 `RTK导航优化` 分支维护，`main` 继续作为稳定集成主线。

## 0.25 m 膨胀候选与避障参数

以任务工作区安装中解析的 RTK 正式 `map` 和配准 `odom` launch 为准。2026-11-24 将两个档位的全局和局部膨胀半径统一为 0.25 m。

| 参数 | 正式导航与配准 |
| --- | --- |
| 膨胀半径、衰减系数 | 全局和局部均为 0.25 m、4.5 m⁻¹ |
| 全局路径代价门槛 | Dijkstra 排除代价 174 及以上的已知格，平滑和复用上限 173。按 0.14 m 足迹内切半径和衰减计算，路径中心到障碍格中心的理论门槛约 0.222 m，取格受 0.05 m 分辨率影响 |
| 车体足迹 | 长 0.345 m、宽 0.28 m，前伸 0.20 m、后伸 0.145 m、左右各 0.14 m。按 0.22 m 实车宽度名义每侧余量 0.03 m |
| 地图与传感器 | 局部滚动 9 × 9 m，正式全局 40 × 40 m，配准全局 24 × 16 m。两张地图的雷达标障和射线清除均为 4.5 m。全局可暂规划到未知区域，局部 Guard 拒绝足迹进入未知、占用或地图外区域 |
| Guard | 制动减速度参数 0.20 m/s²，最短预测 0.80 s，指令、TF、LIO 各过期 0.50 s，局部地图过期 1.50 s。正式档可按制动包络降速。以 0.8 m/s 直行为例，全速前向扫掠检查时间为 4.5 s、中心位移 3.6 m，另计整车足迹 |
| DWA | 正式档上限 0.80 m/s、45°/s，配准档 0.35 m/s、约 28.6°/s。轨迹预测 1.7 s，障碍权重 0.01，足迹缩放起点 0.25 m/s，最大比例 20% |

RTK DWA 的 Guard 使用实际局部代价图和车体足迹。配置模板中的 `min_obstacle_dist=0.144` 属于 TEB，`front_stop_distance=0.15` 属于另一套前向雷达 Guard。这里的路径中心门槛、软件外形和预测距离分别描述规划与控制的检查。实车最小净空仍需实测。

独立候选增量构建及原生安装检查通过，7 项定向 launch 测试通过。两个档位的完整展开参数位于任务工作区 `logs/inflation-025/params-map.yaml` 和 `logs/inflation-025/params-odom-preparation.yaml`。隔离 ROS 验证了初始未知的 6 m 目标，开阔场景用时 10.70 s，绕墙场景用时 47.01 s，两次的 Mux 速度指令峰值均为 0.800 m/s。日志为同目录 `straight.log` 和 `detour.log`。

## 11 月 24 日启动与速度复测

- 最新实车候选会话仍在 `odom` 准备档，DWA 和 Guard 的上限均为 0.35 m/s。6 个 RViz 选点都发在 `odom`，最后一次导航在反复振荡后停止。证据位于任务工作区 `logs/ros/log/rtk-navigation-odom-e5t8rlyx.log` 和对应 `7c09ea5a-e80e-11f1-81e5-3c6d661ed63d/rosout.log`。这次没有同步速度遥测。操作者确认电机实测可达 0.8 m/s，记录范围为其电机试验。
- 正式档保留 0.8 m/s、45°/s 和 0.20 m/s² 制动参数，前进加速度上限调至 0.50 m/s²。DWA 提高进度评分并加入零角速采样。墙边基线诊断显示主要耗时来自 DWA 的慢速与反复转向。全局规划用私有地图副本在 0.45 m 内增加软偏好，让路径更早绕开墙端，实际全局和局部膨胀半径仍为 0.25 m。局部障碍评分、碰撞足迹与 Guard 制动检查沿用已核实的设置。
- 独立原生安装检查、336 项离线用例、7 项 launch 配置检查以及两项 C++ 几何与起步转向检查通过。隔离 ROS 从 0.8 rad 的初始朝向完成纯转向，开始前进时方位误差 0.070 rad。6 m 和 12 m 开阔任务的 Mux 速度指令峰值都是 0.800 m/s，达到 0.7 m/s 的任务时间占 52.1% 和 70.6%。
- 同一遮挡墙模拟在过墙路段（x 为 3.5–5.3 m）有 93.9% 的时长达到 0.4 m/s，69.7% 达到 0.5 m/s，Guard 在需要时拦停。整个 6 m 绕墙目标在 20.73 s 到点。证据位于任务工作区 `logs/start-heading/final-heading.log`、`final-straight-6m.log`、`final-straight-12m.log` 和 `final-wall.log`。 重复试验在 17.04 s 到点，过墙段全程达到 0.5 m/s，逐帧检查软件足迹未和模拟墙重叠，见 `wall-footprint.log`。
- 用真实软件导航链和合成传感器执行往返，回程停在距起点 0.106 m 处，随后完成正式 `map` 切换。后方加障碍的独立试验中，Guard 拦截并停车，返程报告失败。Qt 回环 ROS 验证了 Waypoint 显示、取消不发目标、延迟确认提交原位置以及 Mux 停车时拒发。证据见任务工作区 `logs/start-heading/roundtrip-map.log`、`/tmp/rtk-roundtrip-registration/rear-wall.log` 和 `/tmp/slicer-rtk-startup-desktop/waypoint-proof.log`。
- 实车的可持续速度取决于路径、可见制动距离和实际底盘响应。对于确需小半径转弯、观测不足和停车区域，Guard 按实际可用空间处理。现场复测需记录原始速度、Guard、Mux、编码器、LIO 和 MCU 状态。

## 11 月 24 日桌面简化与闭环对向复测

- 旧候选用普通 `odom` 送 2D Nav Goal 时仍加载 0.35 m/s 配准档，见任务工作区 `logs/ros/log/rtk-navigation-odom-ksx0xi3a.log`。本次从桌面分开普通巡航和配准档。0.8 m/s 上限、0.50 m/s² 前进加速度、0.25 m 膨胀半径和 0.20 m/s² 制动模型保留。DWA 前进档采样从 7 增至 9，进度权重从 1.2 增至 1.4。
- 独立 Catkin 构建、原生安装检查、338 项离线用例、8 项 launch 检查和 C++ 闭环转向用例通过。桌面测试单独运行 73 项通过。设备未连接到这些测试会话。证据见任务工作区 `logs/local-path-start/` 的 `build-final.log`、`native-final.log` 和 `offline-final.log`。
- 从 0.80 rad 初始朝向发 6 m 目标，普通 `odom` 与正式 `map` 在开始前进时的误差分别为 0.104、0.099 rad，Mux 速度指令峰值都为 0.800 m/s，两次都到点。最终版的合成绕墙目标用时 18.41 s，x 为 3.5–5.3 m 的绕墙段全程达到 0.5 m/s，约 68% 时长达到 0.7 m/s。Guard 在其他路段曾按扫掠范围拦停。证据为同目录 `odom-turn.log`、`map-turn.log` 和 `wall-final.log`。
- 独立参数对比的 6 m 墙场景中，绕墙段达到 0.5 m/s 的占比由约 84% 提高到 100%。同一候选参数的 12 m 开阔场景，约 73% 时长达到 0.7 m/s。A/B 对比见 `/tmp/rtk-dwa-tuning/baseline-wall.log`、`progress-14-wall.log`、`progress-14-open-12.log`。
- `check_auto_registration_ros.py cli --odom-cruise` 验证先按普通档启动，WGS84 提交后切入配准，再完成原 `map` 目标。见 `odom-to-wgs84-final.log`。桌面调用的配准服务也已验证在导航空闲时切入 0.35 m/s 档，见 `preparation-service.log`。当前会话尚未验证真实 RViz 画面、实车闭环转向、全套桌面往返配准和实际持续速度。正式车端发布仍独立安装。

## 开发、现场复测和回退

首次构建按车端入口说明（`~/workspace/slicer/START_HERE.md`）从任务根导入 `dependencies.repos` 的固定依赖，运行 `bash tools/build_native.sh "$W" -j2`，目录变量 `W=~/workspace/slicer/dev/rtk-navigation-optimization`。构建后在任务源码根下用 `SLICER_NATIVE_WS="$W" ./tools/rtk_desktop_ui.sh` 启动任务安装。现场由同一执行会话结束已有导航控制链，再操作候选。参数位于 [RTK 导航 launch](../../src/ugv_bringup/launch/slicer_field_navigation.launch)、[正式速度档](../../src/ugv_bringup/config/dwa_field.yaml)和[配准速度档](../../src/ugv_bringup/config/dwa_registration.yaml)。观察 `/cmd_vel/nav_raw`、`/cmd_vel/nav`、`/cmd_vel`、`/slicer_nav_arc_guard/hold_reason`、`/geographic/status` 与 `/geographic/navigation_runtime`。

接手先核对任务工作树和 `~/workspace/slicer/run status`，随后复测空旷直线、直角和窄道停车净空，再复测未扫目标、自动配准往返和 Waypoint 经纬度。退出任务桌面或启动链后，从 `~/workspace/slicer` 用 `./run status`、`./run rtk` 返回既有正式发布。

### 接手提示词

读根 `HANDOFF.md` 与本文件，核对 `RTK导航优化`、`current`、ROS overlay 和现场权限。沿用任务工作树，先复核直线速度与净空，再记录首次未知区目标、配准耗时及 Waypoint 实测证据。
