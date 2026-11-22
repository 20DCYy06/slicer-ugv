# 正式视觉抓取入口与双模式导航提速 · 2026-11-22

- 从 `fa8726b` 在 `fix/rtk-desktop-standard-grasp` 独立工作区开发。原工作区已有无 RTK 的 `max_forward: 0.8`、`max_reverse: 0.5` 和 `.vscode/`。现场保留倒车 0.5 m/s 与用户视觉标定，源码的默认倒车参数延续原版本。
- 桌面第三步改用 `rosrun slicer_visual_grasp run.sh --run --headless --pick-on-start`。可执行窗口和桌面使用同一 `Runtime.pick()`、`Runtime.release()`。无窗口入口等相机可用后执行正式动作，并通过现有 `GRASP_STATE` 回传。抓取完成可持物继续导航，第二次点击释放。
- RTK 的 DWA 和无 RTK 的 TEB 均限前进 0.8 m/s、线加速度 0.4 m/s²。两套 Guard 的制动距离仍基于 0.20 m/s²。无 RTK 的 TEB 局部规划距离从 1.5 m 调到 3.0 m，局部地图从 5×5 m 调到 7×7 m。导航都用 0x16 车体闭环，并通过 0x18 设置本会话输出上限。900% 按 900‰ 理解，当前整数档配置 91%×固件上限，ACK 期望 1429/1600 ticks，即 89.3125% 占空比。
- 隔离 Catkin 安装、53 项桌面测试、视觉 55 项通过和 1 项缺少录制帧的跳过、15 项 RTK Guard 与 16 项无 RTK 驱动测试完成。Xvfb/ROS 桌面回归通过。合成环境的 RTK odom、RTK map、无 RTK TEB 都在 7 m 直行上达到 0.800 m/s，正常到点停车。无 RTK 速度试验使用已知地图，另外用真实 Gmapping 和模拟雷达完成建图、障碍标记及重置回归。RTK 障碍与超时保护通过。实车峰值和抓取动作仍待现场验证。
- 已部署到 `/home/ugv/slicer_noetic_c30d_v17`。关闭旧桌面或导航后重新执行 `./tools/rtk_desktop_ui.sh` 或 `./tools/no_rtk.sh nav`。回退时关闭相关节点，按相对路径恢复该安装下 `nav-visual-backups/standard-grasp-20261122.vytbqo9e`，并移除该备份 `new-files` 记录的新增文件。源码回到 `fix/rtk-desktop-grasp-independent`。
