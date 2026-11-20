# 融合手柄串口排查 · 2026-11-20

开始时位于 `fix/c30d-udev-replacement` 的 `5707f24`，已有未跟踪的 `.vscode/`。车端 `/dev/c30d` 已由 udev 指向 USB 序列号 `5CF0016377` 的替换板。

`./tools/manual/run_gamepad.sh` 在 AirGrip 初始化报 EACCES。AirGrip 的 by-id 路径实际指向 `/dev/ttyACM0`，权限为 `root:dialout`、`0660`，当时 `ugv` 不在 `dialout` 且 AirGrip 设备没有为该账号设置 ACL。融合底盘配置还引用旧板，因此改为使用 `/dev/c30d` 并核对替换板序列号。配置位于 `tools/manual/combined_gamepad/combined_hardware.yaml`，返工时可单独还原该配置。此项影响独立融合手柄入口。

验证包括 `./tools/manual/run_gamepad.sh --test` 的 11 项离线测试，以及车端底盘身份解析 `/dev/ttyACM2 / 1a86:55d4 / 5CF0016377`。现场 AirGrip 查询可用 `./tools/manual/run_airgrip.sh --check` 执行。无需 Catkin 构建或 ROS 回放。

现场人员在桌面终端完成管理员认证，仅对当前 AirGrip 设备节点授予 `ugv` 临时读写 ACL。首次 PING 超时，随后原始查询收到合法心跳及 PING、GET_STATUS 应答，标准检查入口连续三次通过。AirGrip 固件回复 `1.2T.1`、`pump=0`、`system_state=RELEASE`。手柄节点可读，底盘和 AirGrip 串口可读写。未启动融合控制或发送泵动作。AirGrip 重新插拔后需要重新授予临时 ACL。

## 直行方向

现场报告正向手柄指令时车辆后退，日志记录 `vx=+0.052` 和 `+0.041`、`wz=0`。首次手推采样在 Motor OFF 且四轮 PWM 为零时观察到 B 编码器几乎无变化。复测日志 `tools/hardware/logs/feedback_20261120T155450_165159Z.jsonl` 中，275 帧车轮遥测均为 Motor OFF、动力断开、PWM 全零。向前推时 A/B/C/D 原始计数增量为 `[-227294, 225329, 229306, -231364]`，B 已恢复。

现场随后确认 C/D 两个前轮方向正确、A/B 两个后轮方向相反。融合手柄配置成对反转 A/B，`pwm_signs` 为 `[-1, 1, -1, -1]`，`encoder_signs` 为 `[-1, 1, 1, -1]`。离线回放使用新配置时，四轮按上述顺序均为正向，距离依次为 `[0.7736, 0.7669, 0.7804, 0.7874]` 米。程序重启握手时下发极性。需要回退时还原融合手柄配置文件中的两组极性并重启程序。尚需现场在架空、可急停条件下以最低档复测前后和左右。
