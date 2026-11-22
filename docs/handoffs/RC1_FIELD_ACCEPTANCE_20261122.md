# 2026-11-22 候选发布实车切换

## 版本和启停

候选为 `~/workspace/slicer/releases/20261122-workspace-rc1`，整车完整提交
`adb4313b0fb7c825e040fd25b7f1cc61624a26df`。发布的硬件、FAST-LIO、Livox 驱动与 SDK2
分别核对为 `a377b45c`、`88c50b57`、`470b3988`、`c0796f04`，完整 SHA 在
`release.json` 与 [dependencies.repos](../../dependencies.repos)。候选实际解析的应用包
均来自其 `install/share`，Noetic 底层来自 `/opt/ros/noetic`。22 份运行配置摘要、
现场硬件输入摘要及随包固件镜像摘要均匹配。现场使用 `/dev/c30d`、已配置的 GNSS
稳定串口和 MID360 既有以太网连接。NoRTK 的 `max_reverse=0.5` 沿用部署值。固件镜像
版本 v1.7，板端读回属于既有记录。本轮未重新刷写或改标定。

开工时新 `current` 不存在，旧 `~/slicer_noetic_current` 指向 `~/slicer_noetic_c30d_v17`。
车端没有旧 ROS Master、导航、driver 或串口占用。换电后原 MID360 网口恢复。按维护手册
建立 `current`，用 `run status` 和 `run shell rospack find …` 核对实际选择。现场仅
运行一套导航和串口所有者，每次换模式前检查退出。实车核心通过，新工作区已接管。验收后 NoRTK、RTK 及手动测试工具均已退出，保留候选选中。见根 [HANDOFF](../../HANDOFF.md)。

## 现场证据

证据目录为 `~/workspace/slicer/logs/20261122-rc1-field-acceptance/`。下表仅记本次
现场结果，候选的历史软件验证另见[多仓交接](WORKSPACE_SPLIT_20261122.md)。

| 项目 | 结果及证据 |
| --- | --- |
| RTK 已安装入口和 ROS Master | 无参数 `run rtk` 在本地 X11 启动和退出，未打开 driver。发布选择器传递 `--help` 时会触发 Catkin setup 的参数解析报错，正常启动使用无参数入口。随后以 `run shell roslaunch ugv_bringup slicer_field_sensors.launch` 和 `slicer_navigation_runtime.launch` 启动实车传感器、地理服务、导航和 Guard/Mux。本轮 RTK 栈不启动 driver，Mux 保持 `manual_hold=true`、`hold_reason=startup`，采样输出全为零。见 `rtk-local-desktop.txt`、`rtk-node-list.txt`、`rtk-hold.txt`。 |
| MID360、FAST-LIO、局部 EKF | 两种模式均收到点云、IMU、FAST-LIO 里程计和 `/odom`。对精确 TF 边按 `callerid` 采样，唯一 `odom → base_link` 为 `/slicer_ekf`。NoRTK 唯一 `map → odom` 为 `/slam_gmapping`。见 `rtk-passive-topics.txt`、`nortk-stationary-topics.txt`。 |
| C30D、Guard、Mux、停止 | 首次 NoRTK 的 driver 报底盘响应超时，required 节点退出后全栈停机。候选和旧源码工具均可读板端状态，无故障锁存，PWM 和轮速均为零。随后候选握手四项 ACK、轮速、车体 IMU 和 500 ms 看门狗声明均正常。重试 driver 就绪，`/cmd_vel` 仅由 Mux 发布。在架空条件下，0.08 m/s 短输入经 Guard/Mux，轮速实测非零，撤除后输出和轮速归零。没有注入 watchdog 故障。见 `nortk-attempt1.log`、`c30d-handshake.txt`、`nortk-wheel-off-ground.txt`。 |
| NoRTK 建图和导航 | Gmapping 真实出图，B 样条查询返 11 点，TEB 由正式 launch 加载。手动工具退出后 NoRTK 重新接管。按现场净空确认，以会话内 TEB 前进上限 0.10 m/s 提交 0.40 m 地面目标，Action 返回 `SUCCEEDED`，估算位移 0.275 m，停车后指令和轮速归零。限速随停栈失效，未写发布配置。见 `nortk-make-plan.txt`、`nortk-ground.txt`。 |
| 手动接管 | 导航停栈后启动 `run shell ./tools/manual/run_gamepad.sh --drive-only`。现场操作，地面低速上限 0.10 m/s，有前进和转向，末次指令归零，现场确认停止。组合手柄独占底盘与气夹，Y/A 触发 PB14 舵机、松开回 1500 µs，X/B 空载吸放有设备 ACK，现场确认正常。键盘 `--check` 读到零轮速及零 PWM，键盘实际运动待测。见 `gamepad-ground.txt`、`combined-gamepad.txt`、`wasd-readonly.txt`。 |
| 正式视觉 | 候选安装的正式视觉 `--check` 通过。用安装模块和现场相机配置取到 640×480 帧，YUYV 30 fps。正式入口无参数预览在本机 X11 打开相机，退出后释放设备。见 `vision-check.txt`、`vision-camera.txt`、`vision-preview.txt`。 真正抓取工件和完整释放流程待测，空载吸放见组合手柄结果。 |
| GNSS 与自动配准未就绪 | 当天无差分账号，使用受保护的串口验收配置关闭 NTRIP，保持原现场文件与定位门槛。GNSS 串口、NMEA、GGA 和状态反馈正常。采样时 GGA q=0、`fix_valid=false`，地理状态 `raw_quality_not_fixed`，未建立配准。见 `gnss-status.txt`、`rtk-geographic-cli.txt`。首 WGS84 自动配准、全局纠偏、地理到点与复核及飞机联调待验收。 |

## 验收后的归档

车端旧源码、历史工作树和 v1.7 安装已统一移入
`~/workspace/slicer/archive/legacy-vehicle/`。已结束的任务工作区位于
`archive/completed-dev/`。当前发布仍为 RC1。以下回退记录反映首次切换时的状态，
历史复现前先查阅本机归档目录下的说明并恢复原路径。

## 首次切换的回退记录

`~/workspace/slicer/START_HERE.md` 是车端操作表。旧源码 `~/slicer-ugv` 原地标为
LEGACY，已有未提交文件、活动终端及引用其 Git 公共目录的历史工作树。旧 v1.7 安装原地
保留。较早的原生安装与已结束的整理工作区保留用于历史证据和固定路径构建复现；清理记录见
[多仓交接](WORKSPACE_SPLIT_20261122.md)。本轮没有移动 Git 公共目录。

若候选异常，先用当前模式停止并确认底盘串口无人占用，再恢复选择。首次切换前新
`current` 不存在，确认它仍指向本候选后可 `unlink ~/workspace/slicer/current`。
用新终端和旧源码 `~/slicer-ugv` 配合 `SLICER_NATIVE_WS=$HOME/slicer_noetic_current`
运行其 `tools/no_rtk.sh nav --background` 或 `tools/rtk_desktop_ui.sh`。已只读检查
旧链接、脚本、overlay、两种模式的 launch 节点解析和 C30D 反馈，旧整车运行链本轮未重启。回退目录不会恢复固件、
标定或现场配置，这些沿用各自记录。

## 后续验收

- 恢复 RTK Fixed 后做自动准备、全局纠偏、WGS84 到点及地理复核，再联调飞机。
- 补键盘实际行驶、正式抓取工件和看门狗故障锁存恢复。若冷启动再次出现底盘超时，留存启动和握手阶段日志。
