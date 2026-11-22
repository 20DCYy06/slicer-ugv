# 视觉抓取每秒反馈对准

- 在 `fix/visual-grasp-periodic-feedback` 的独立工作区开发，基线更新到 `5026df4` 以保留 RTK 桌面抓取联动。原检出中的无 RTK 现场配置修改和 `.vscode/` 未纳入本次提交。v1.7 仅更新视觉包和现场视觉配置。
- 开始前确认气泵空闲并取得新绿框。移动时约每 1 s 读取新帧重算吸嘴红点到绿框中心的位移。丢框时继续跟踪最后有效目标的剩余位移。初定对准或计划完成后，使用连续新底盘反馈确认停稳，再用停稳之后的新相机帧复核。复核帧无绿框或红绿距离不超过阈值才进入开泵和下放。可见偏差超限时继续对准，相机错误、过期反馈和对准超时均中止。
- 现场 `alignment.tolerance_px` 为 8 像素，`grasp.max_lower_s` 为 20 秒。移除包内和现场配置的 `snapshot_move.approach_extra_m`，即使旧自定义配置仍带该键也不会产生固定补偿。观察 `[VISION_UPDATE]`、`[VISION_OCCLUDED]` 和 `[VISION_ALIGNED]`。操作见 [视觉抓取说明](../../src/slicer_visual_grasp/README.md)。
- 独立 Catkin 安装和现场 v1.7 都通过依赖检查和模拟回归。两边各执行 54 项测试，53 项通过，1 项缺少录制帧而跳过。现场安装在隔离 Xvfb 中通过 12 项假设备界面测试。本次未向实车发送动作指令。
- 现场回退备份位于 `/home/ugv/slicer_noetic_c30d_v17/visual-grasp-backups/periodic-feedback.dk5JvTEH`。停止视觉窗口后，可恢复备份中的 `lib/`、`share/` 对应安装路径，按 `new-files` 移除新增文件，并用 `vehicle-config.json` 恢复外部用户配置。
