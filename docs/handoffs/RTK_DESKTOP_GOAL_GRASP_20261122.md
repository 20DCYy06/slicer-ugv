# RTK 桌面目标替换、抓取与实车限速诊断 · 2026-11-22

- 从 `355faa6` 在 `fix/rtk-desktop-goal-grasp` 独立工作树开发。原运行检出已有 `src/ugv_bringup/config/no_rtk/slicer_config.yaml` 的修改与未跟踪的 `.vscode/`。本轮涉及 RTK 桌面和共用视觉包的无窗口入口。现场日志的底盘驱动来自 `/home/ugv/slicer_noetic_c30d_v17/install`，本轮隔离桌面验证使用 `/home/ugv/slicer_noetic_current`，均沿用现有编译产物。
- 同一桌面再次选择目标时，`move_base` 收到新 Action 目标并替换旧目标。等新鲜状态期间只保留最新点击。「正在前往」对应最近发送的目标。其他控制端的目标、空地任务及 Mux 停车继续保护。
- 视觉抓取位于第 3 步。导航关闭且轮速里程计确认停车时，桌面先关闭自己启动的驱动，确认其进程和轮速发布端退出后启动后台抓取。成功释放或在动作前启动失败时，桌面自动恢复刚暂停的驱动。待释放的抓取保持恢复状态。一键全部停止取消交接。外部终端启动的驱动由原终端操作。
- 现场日志 `~/.ros/log/rtk-desktop-odom-y_ftdfer.log` 记录 Guard 的直行上限 0.800 m/s、转向 0.785 rad/s，并记录两次扫掠范围遇到障碍、未知区或地图边界停车。同一 Master 的 `rosout.log` 有 27 次 DWA 找不到有效路径。DWA 与 Guard 的直行加速度均为 0.20 m/s²。理想情况下从停车加速到 0.8 m/s 约需 4 秒和 1.6 m，计入同等制动距离约需 3.2 m。上一轮隔离 ROS 7 m 直行输出达到 0.800 m/s。
- 现场日志 `~/.ros/log/rtk-desktop-driver-ickbpcvv.log` 确认底盘使用 `body_speed` 的 `0x16` 车体速度命令。600 mm/s 轮速字段和 600‰ 主机 PWM 字段在该模式下不构成上限。v1.7 固件源码的车身直行上限为 1.0 m/s，转弯时仍受各轮限幅。现有日志没有同步的速度命令、轮速和 PWM 样本，无法判断电机死区、供电、打滑或 PI 跟踪是否也拖慢实车。本轮保留现有速度和控制器参数。
- 现场可在同一 ROS 环境用 `python3 tools/rtk/collect_odom_test.py --duration 30 --navigation` 旁路记录一次受控直行。对齐 DWA raw、Guard、Mux、`0x16`、轮速、车身反馈与 PWM。若 raw 已低，先看 DWA 与 `/odom`。若 Guard 才下降，查停机原因与地图。若 `0x16` 已给足而轮速跟不上，再用 PWM 有无饱和区分可用驱动力和控制器响应。
- 静态检查、51 项桌面测试和 5 项后台抓取测试通过。独立 ROS Master、Xvfb 与 RViz 实际鼠标事件验证了快速点击只发最新待发目标、正在执行时可发送替换目标及其他控制端目标优先。串口交接使用模拟进程和反馈验证。未发布实车运动命令，也未对实车抓取或速度做试车。
- 从 `/home/ugv/slicer-ugv` 重开 `./tools/rtk_desktop_ui.sh`。操作见[整车操作手册](../OPERATIONS.md#rtk-本地分屏桌面)。回退到 `fix/rtk-desktop-integration` 即恢复先前桌面。
