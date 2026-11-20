# 融合手柄串口排查 · 2026-11-20

开始时位于 `fix/c30d-udev-replacement` 的 `5707f24`，已有未跟踪的 `.vscode/`。车端 `/dev/c30d` 已由 udev 指向 USB 序列号 `5CF0016377` 的替换板。

`./tools/manual/run_gamepad.sh` 在 AirGrip 初始化报 EACCES。AirGrip 的 by-id 路径实际指向 `/dev/ttyACM0`，权限为 `root:dialout`、`0660`，当时 `ugv` 不在 `dialout` 且 AirGrip 设备没有为该账号设置 ACL。融合底盘配置还引用旧板，因此改为使用 `/dev/c30d` 并核对替换板序列号。配置位于 `tools/manual/combined_gamepad/combined_hardware.yaml`，返工时可单独还原该配置。此项影响独立融合手柄入口。

验证包括 `./tools/manual/run_gamepad.sh --test` 的 11 项离线测试，以及车端底盘身份解析 `/dev/ttyACM2 / 1a86:55d4 / 5CF0016377`。现场 AirGrip 查询可用 `./tools/manual/run_airgrip.sh --check` 执行。无需 Catkin 构建或 ROS 回放。
