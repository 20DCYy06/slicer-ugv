# RTK 导航优化

任务分支为 `RTK导航优化`，从当时的 `origin/main`（`a7a64c70698d07779b1be77ae7059fcc877245b9`）创建。工作树是 `~/workspace/slicer/dev/rtk-navigation-optimization/src/slicer-ugv`。现场诊断依据是 2026-11-23 速度排查交接（`dev/rtk-field-speed-audit/src/slicer-ugv/docs/handoffs/RTK_FIELD_SPEED_AUDIT_20261123.md`）和 `~/workspace/slicer/data/rtk-corner-20261123-084654/analysis/speed-chain/`。硬件依赖为 `a377b45c48c4be91add25b73eacedabdb8c264de`，固件改动为零。任务验证使用独立安装和回环 ROS master。正式 `current` 仍是 `20261123-rtk-fixed-a7a64c7`。

## 变更

- RTK 全局、局部膨胀半径由 0.24 m 增至 0.48 m。两张地图均在 4.5 m 内标障和清除，使全局规划与局部控制尽早使用同一批新障碍。车身按现场给出的 22 cm 计，现有软件横向足迹宽 28 cm，名义上每侧多 3 cm。膨胀距离从障碍格向外计，扩大高代价区域。全局搜索、路线复用与平滑共用已观察障碍的代价门槛，约相当于路径中心离障碍格 0.23 m，分辨率会影响落在哪一格。Guard 的硬足迹维持原尺寸。旧回放最近点云距软件足迹约 5 cm，不能据此推定所有路况的最小物理净空。要提高强制最小净空，需单独量车体外缘及传感器误差并复测直角和窄道。
- RTK 复用 NoRTK 的 Dijkstra 与 B 样条规划器，使用连续梯度回溯以更早转向，随着扫描推进分段平滑路径的已观察前缀，其余段保留临时规划，Action 终点仍是原目标。已有路线仍有效且代价相近时继续走该路线，新增障碍阻断时重规划。RViz `Nav Goal` 允许点击全局网格内的未知区，局部 DWA 和 Guard 按已扫清的可行空间行驶。缺少可通行的局部路线时停车，目标超出全局地图网格时无法规划。配准选择的候选点和路线仍须通过已观察区域检查。
- 正式导航上限保留 0.8 m/s 和 45°/s。DWA 将前进速度采样从 3 个增加到 7 个，并提高跟踪全局绕障路线的权重。Guard 在原指令过快、实际 LIO 位移及上一帧指令均可在已观察范围内制动时，沿现有前进弧线寻找较低可用速度，并按每个候选速度单独校验制动包络。制动净空不足、输入过期、规划无可用前进指令时停车。底盘和固件参数不变。
- 配准 DWA 的线速上限由 0.25 m/s 增至 0.35 m/s，角速度、加速度、质量门和停车确认沿用原值。拟合增量处理新配对，样本淘汰或重置时重建缓存。

## Waypoint 打点

1. 在 RTK 桌面执行第 1 步「启动定位与传感器」。等新鲜 RTK Fixed，车辆停稳约 1 秒，点顶部「记录参考点」。
2. 人工移动至少 2 米，间距在场地允许时可更大。再次等 Fixed 和停稳，点「记录参考点」。两点使用各自同历元的 Fix 和局部里程计及已配置的天线偏移。
3. 选「Waypoint 打点」，在 RViz 中点位置。右侧显示 WGS84 纬度、经度，可右键复制。未完成正式配准时标为「估算」，离参考点越远越受基线和里程计误差影响。已有正式配准时优先使用地图转换。桌面重启、RTK 会话改变或里程计断流/重置后需重新录点。打点不会发送导航任务。

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

## 开发、现场复测和回退

首次构建按车端入口说明（`~/workspace/slicer/START_HERE.md`）从任务根导入 `dependencies.repos` 的固定依赖，运行 `bash tools/build_native.sh "$W" -j2`，目录变量 `W=~/workspace/slicer/dev/rtk-navigation-optimization`。构建后在任务源码根下用 `SLICER_NATIVE_WS="$W" ./tools/rtk_desktop_ui.sh` 启动任务安装。现场由同一执行会话结束已有导航控制链，再操作候选。参数位于 [RTK 导航 launch](../../src/ugv_bringup/launch/slicer_field_navigation.launch)、[正式速度档](../../src/ugv_bringup/config/dwa_field.yaml)和[配准速度档](../../src/ugv_bringup/config/dwa_registration.yaml)。观察 `/cmd_vel/nav_raw`、`/cmd_vel/nav`、`/cmd_vel`、`/slicer_nav_arc_guard/hold_reason`、`/geographic/status` 与 `/geographic/navigation_runtime`。

接手先核对任务工作树和 `~/workspace/slicer/run status`，随后复测空旷直线、直角和窄道停车净空，再复测未扫目标与两点经纬度。退出任务桌面或启动链后，从 `~/workspace/slicer` 用 `./run status`、`./run rtk` 返回既有正式发布。

### 接手提示词

读根 `HANDOFF.md` 与本文件，核对 `RTK导航优化`、`current`、ROS overlay 和现场权限。沿用任务工作树，先复核直线速度与净空，再记录首次未知区目标、配准耗时及 Waypoint 实测证据。
