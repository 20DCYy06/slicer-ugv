# C30D 统一锁定和清锁命令 · 2026-11-21

- 从 `fdad0cb` 在独立工作树修复。已有无 RTK 参数和 VS Code 文件的本地编辑。板上固件继续使用已验收的 v1.7。
- `Hardware/LockDown/lock.py` 和 `Hardware/Unlock/unlock.py --unlock` 按 USB 序列号寻找同一用户的底盘驱动。本机 Unix socket 将请求送到唯一串口所有者。串口空闲时脚本直接操作。
- 测试锁定要求 12 V 动力、Motor ON、四轮静止和零 PWM。导航驱动确认零速度帧已被 MCU 接收后，再发送测试锁存帧。挂机锁存继续保持。导航驱动在确认通信类故障的清锁条件后异步调用 `unlock.py --unlock`，确认完成后才接受新速度。
- 用模拟 MCU 验证两个命令、真实子进程和 Unix socket。旧版 RTK 独立底盘驱动的实车恢复已于本轮之前通过用户验收。新命令的带动力实车验收留待现场。命令与结果见 [操作手册](../OPERATIONS.md#底盘通信锁存恢复与测试)。
