# C30D 导航通信锁存恢复 · 2026-11-21

- 从 `fix/c30d-udev-replacement` 的 `78f64e9` 在独立工作树实施。原检出已有无 RTK 速度参数修改和未跟踪的 `.vscode/`，本任务不提交这些文件。
- 无 RTK、RTK 的 0x16 模式和独立解锁工具共用通信锁存恢复。固件确认 250 ms 连续零 PWM、车轮静止、电源和使能有效后才清锁。自动恢复仅清理 UART、导航指令看门狗和测试锁存。恢复后需要新速度指令。
- v1.7 源码和镜像已归档。ARM GCC 9.2.1 生成 11876 字节镜像，SHA-256 为 `4e9bcafeab040d7f1bd0ad8758948bc180e1c231afa247f8c492fd8a21e4431a`。固件原生、SLC2 协议、ARM 构建及 16 项硬件工具、16 项无 RTK 和 68 项 RTK 驱动离线测试通过。
- 用户确认断开 12 V 后升级替换板 `5CF0016377`。写前双备份逐字节一致，仅写入 sector 0。整片读回 SHA-256 为 `81067fa115fdcc426f6b4d3e2ef50517ef6767a4f442877db54052e45c4d7153`，选项字节校验通过。备份和日志位于 `Hardware/Backups/board_5CF0016377_1795270966021742526/`。
- USB 供电下验证恢复能力位、IMU 就绪、26 帧零 PWM、八个 CCR 全零和 982‰ 握手。零输出记录为 `Hardware/Logs/zero_check_1795271241424849403.json`。带动力锁存与恢复待现场验收。
- ROS 安装位于 `/home/ugv/slicer_noetic_c30d_v17`，构建和 `tools/check_native.sh` 检查通过。独立 ROS master 上用模拟 MCU 验证实际 `rosservice call` 返回锁存确认，随后恢复。操作见 [整车操作手册](../OPERATIONS.md#底盘通信锁存恢复与测试)，回退见 [v1.7 说明](../../Hardware/Firmware/v1.7/README.md)。
