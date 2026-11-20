# 融合手柄串口排查 · 2026-11-20

开始时位于 `fix/c30d-udev-replacement` 的 `5707f24`，已有未跟踪的 `.vscode/`。车端 `/dev/c30d` 已由 udev 指向 USB 序列号 `5CF0016377` 的替换板。

`./tools/manual/run_gamepad.sh` 在 AirGrip 初始化报 EACCES。AirGrip 的 by-id 路径实际指向 `/dev/ttyACM0`，权限为 `root:dialout`、`0660`，当时 `ugv` 不在 `dialout` 且 AirGrip 设备没有为该账号设置 ACL。融合底盘配置还引用旧板，因此改为使用 `/dev/c30d` 并核对替换板序列号。配置位于 `tools/manual/combined_gamepad/combined_hardware.yaml`，返工时可单独还原该配置。此项影响独立融合手柄入口。

验证包括 `./tools/manual/run_gamepad.sh --test` 的 11 项离线测试，以及车端底盘身份解析 `/dev/ttyACM2 / 1a86:55d4 / 5CF0016377`。现场 AirGrip 查询可用 `./tools/manual/run_airgrip.sh --check` 执行。无需 Catkin 构建或 ROS 回放。

现场人员在桌面终端完成管理员认证，仅对当前 AirGrip 设备节点授予 `ugv` 临时读写 ACL。首次 PING 超时，随后原始查询收到合法心跳及 PING、GET_STATUS 应答，标准检查入口连续三次通过。AirGrip 固件回复 `1.2T.1`、`pump=0`、`system_state=RELEASE`。手柄节点可读，底盘和 AirGrip 串口可读写。未启动融合控制或发送泵动作。AirGrip 重新插拔后需要重新授予临时 ACL。
