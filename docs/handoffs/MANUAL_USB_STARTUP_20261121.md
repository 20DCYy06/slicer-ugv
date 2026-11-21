# 手动工具 USB 启动修复 · 2026-11-21

- 从 `ae1d2cd` 在独立工作树修复。原 Jetson 检出有无 RTK 速度参数编辑及未跟踪的 `.vscode/`。本次涉及舵机、气夹、组合手柄三个源码入口。
- Jetson 旧会话缺少 `dialout` 进程组，C30D 和 AirGrip 在原入口均报 EACCES。账号已加入该组，串口均为 `root:dialout`、权限 `0660`。三个入口现在为旧会话加载账号已有的组权限。AirGrip 首次 PING 曾超时，同设备后续查询通过。打开串口后仅对 PING 超时重试，最多三次；吸放命令沿用原行为。
- C30D 使用 `/dev/c30d` 及 USB 序列号 `5CF0016377`，AirGrip 使用 USB by-id 及序列号 `de654403579b5b2a`，组合手柄改用 `/dev/input/by-id/usb-2563_USB_WirelessGamepad-joystick`。同一设备换口后由 udev 重新生成别名。断开后退出当前控制会话，插回后重新运行原命令。
- 静态与离线：Shell 语法和 diff 检查通过。舵机自检、AirGrip 8 项、融合手柄 12 项、通用手动工具 14 项通过。无 Catkin 构建、ROS 回放或控制链改动。

操作见 [整车操作手册](../OPERATIONS.md)。回退时撤销本次修复提交；旧会话可用 `sg dialout -c '原启动命令'`。
