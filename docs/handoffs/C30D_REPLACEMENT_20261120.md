# C30D 替换板烧录 · 2026-11-20

Jetson 开始时位于 `migration/native-noetic`，源码提交 `2bcf515`，工作树干净。原安装记录对应 USB 序列号 `5CF0016129`。本次现场换板后，实际烧录目标为 `1a86:55d4 / 5CF0016377`，串口为 `/dev/ttyACM2`。现场确认烧录条件已准备，按断动力、Motor OFF、仅 USB 供电执行。

- 离线复核 v1.4 的 23 个构建输入。使用归档对应的 GCC 10.3.1 和 binutils 2.38 在临时目录构建，本机固件和串口协议测试均通过。11000 字节镜像 SHA-256 为 `b88a173597c584d6a40eff88c59bd1fba4cf7033f516cfdcf3e64fa431e29056`，与原验收镜像相同。
- 替换板烧前双读 512 KiB 一致，SHA-256 为 `2151134fbe6da812b53e8a9a5a6e868710c70150c688ca2d2824188c5ba41e37`。在擦写会话中再次核对整片和选项字节，仅擦写扇区 0。刷后整片 SHA-256 为 `49dc6bd311445e7ab6cf5b63c4a3ee34ab1862186e633b3016dbe26db120ab8b`。扇区 0 外和选项字节逐字节一致。
- 刷后通过 PB14/TIM12_CH1 能力查询和 26 帧零输出验收。IMU 已就绪，四轮 PWM 与八路定时器 CCR 全零，输出上限 80%。本轮无隔离 ROS 构建、回放或带动力运动测试，也未启动 RTK 或无 RTK 驱动。

车端证据位于 `Hardware/Backups/replacement_5CF0016377_v1.4.Vbh2qEc9/`，含双备份、镜像与构建日志、安装记录、整片读回及零输出记录。需要回退时，以其中 `original/flash_read_1.bin` 的前 16 KiB 作为本板恢复源，先核对另一份备份，再在相同断动力条件下恢复扇区 0 并校验整片和选项字节。恢复尚未实刷。

烧录和零速验收在临时目录使用替换板序列号。启用正式 RTK 或无 RTK 控制链前，需按实际部署所用的 overlay 核对正式车体速度配置中的序列号、串口路径和对应控制参数。

## 替换板 udev 别名

`fix/c30d-udev-replacement` 从 `c19fe10` 开始，接手时工作树干净。仓库的 `deploy/udev/99-c30d.rules` 现按 `1a86:55d4 / 5CF0016377` 匹配 `/dev/c30d`。现场开始时没有安装同名规则。静态核对通过，现场人员安装并加载规则后，确认车端规则与提交 `e4c5119` 一致。`/dev/c30d` 实际解析为 `/dev/ttyACM2`，udev 报告序列号 `5CF0016377`。正式车体速度配置仍保留原绑定。此项无需 Catkin 构建或 ROS 测试。

车端安装和加载方式见[依赖说明](../DEPENDENCIES.md#设备和现场配置)。验证时运行 `readlink -e /dev/c30d`，并用 `udevadm info --query=property --name=/dev/c30d` 核对 `ID_SERIAL_SHORT=5CF0016377`。如需回到本次操作前的车端状态，撤下新装的 `/etc/udev/rules.d/99-c30d.rules`，重新加载规则后重新枚举底盘 USB。
