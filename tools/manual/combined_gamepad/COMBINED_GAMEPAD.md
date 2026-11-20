# 底盘 / PB14 或 PB15 / AirGrip 手柄融合

```bash
./tools/manual/run_gamepad.sh
```
左摇杆Y轴油门，右摇杆X轴方向，沿用原方向、死区、加速度及档位范围。
Y=0按住顺时针、A=2按住逆时针；松开停止，同时按下也停止。
X=3按一次吸取、B=1按一次释放（按用户XYBA测试编号）。
方向若反了交换tools/manual/b15_servo/config.json的两个端点。舵机20Hz续期，共用底盘串口，不改变车轮速度映射。
RB=7升档、LB=6降档；终端R/E亦可。线速度0.1–0.8m/s，角速度0.05–0.5rad/s。
B现用于吸口释放，不再用于底盘停止；松开摇杆归零，终端空格停底盘和舵机，泵不变。
Q/Ctrl+C退出：先停止底盘和舵机，再尝试释放本程序启动过的吸取。

支持报告 PB14 或 PB15 能力的 C30D 固件和 AirGrip 1.2T。先退出 ROS 底盘驱动、独立手柄、舵机键盘、AirGrip 及其他同设备的串口进程。
启动只做零速初始化和能力/状态查询，不自动转舵机或吸取；底盘不接受旧指令。
吸口ACK等待放在独立串口线程，不中断50Hz底盘刷新。断开AirGrip物理连接不会自动释放，软件不能保证断线释放。
从工作区根目录运行。配置在 `tools/manual/combined_gamepad/combined_gamepad.json`，引用融合底盘配置、舵机配置和 AirGrip 配置。

融合模式使用 `/dev/c30d`，仍核对底盘的 USB 序列号。AirGrip 使用其 USB by-id 路径，运行账号需要对应设备的读写权限。可以用 `./tools/manual/run_airgrip.sh --check` 单独检查 AirGrip。

无硬件测试：`./tools/manual/run_combined_gamepad.sh --test`。
run_gamepad.sh默认已统一到此融合程序；旧纯底盘使用`./tools/manual/run_gamepad.sh --drive-only`（其B仍为停车）。`--demo/--check/--config`保留旧纯底盘接口；融合自定义配置使用 `./tools/manual/run_combined_gamepad.sh --run --config FILE`。
按键边沿打印[BUTTON]标签，Y/A松开后应见当前引脚标签对应的 1500 us；X/B操作后应见[AIRGRIP_PUMP_ACK]设备确认。

融合手柄专用速度配置：tools/manual/combined_gamepad/combined_hardware.yaml，speed.linear_max=0.8；每档0.05m/s，共15档。
独立 WASD 和纯底盘手柄程序读取 `Hardware/config.yaml`。
