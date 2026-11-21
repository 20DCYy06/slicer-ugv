# C30D 统一锁定和清锁命令 · 2026-11-21

- 从 `fdad0cb` 在独立工作树修复。已有无 RTK 参数和 VS Code 文件的本地编辑。板上固件继续使用已验收的 v1.7。
- `Hardware/LockDown/lock.py` 和 `Hardware/Unlock/unlock.py --unlock` 按 USB 序列号寻找同一用户的底盘驱动。本机 Unix socket 将请求送到唯一串口所有者。串口空闲时脚本直接操作。
- 测试锁定要求 12 V 动力、Motor ON、四轮静止和零 PWM。导航驱动确认零速度帧已被 MCU 接收后，再发送测试锁存帧。挂机锁存继续保持。导航驱动在确认通信类故障的清锁条件后异步调用 `unlock.py --unlock`，确认完成后才接受新速度。
- 通过 24 项硬件工具、16 项无 RTK 和 68 项 RTK 离线测试。用模拟 MCU 验证真实解锁子进程、精确停车 ACK、本机 Unix socket 和退出请求。安装后的 RTK 子进程与无 RTK 模块导入通过。
- 更新 `/home/ugv/slicer_noetic_c30d_v17`，整车安装检查通过。将 `/home/ugv/slicer_noetic_current` 从 v1.6 指向 v1.7。回退默认入口时可把链接指回 `/home/ugv/slicer_noetic_c30d_v16`。旧版 RTK 独立底盘驱动的实车恢复已于本轮之前通过用户验收。新命令待带动力实车测试。命令见 [操作手册](../OPERATIONS.md#底盘通信锁存恢复与测试)。
