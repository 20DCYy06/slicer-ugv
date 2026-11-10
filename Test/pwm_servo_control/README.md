# 8路按住旋转按钮 / 9路PWM位置滑块

```bash
cd /home/potato/Slicer_Workspace2
./Test/run_pwm_servos.sh
# 或后台显示在Jetson远程桌面
./Test/run_pwm_servos.sh --background
```

SSH启动默认使用本机桌面`:0`及现有Xauthority，不使用MobaXterm的localhost:10.0转发；不以root运行窗口。
HID设备当前权限需要sudo，终端首次提示时输入potato的sudo密码。无须安装/修改系统udev规则。
若要使用SSH X11转发，显式加`--x11`；普通桌面终端直接运行同一入口。
依赖系统自带Python3、Tkinter；不需要ROS、pyserial或hidapi。

- **8路**：按住“顺时针”或“逆时针”按钮才旋转，松开、移出按钮、切出窗口或最小化即发送停止。默认1000/1500/2000 μs。
  按住转速由`channel8.hold_speed_percent`配置（默认100%，对应原滑块满幅），不改变PWM端点。
  顺时针的实际对应关系由舵机安装及型号决定；反了就在配置channel8.direction改为-1。
  停不住时微调`neutral_us`。按钮/空格/Esc立即发送8路停止；退出时停止由本程序启动的8路。
- **9路**：角度舵机，默认标称0–180°映射500–2500 μs。
  实际角度范围以舵机型号为准，建议先从中间小范围试动；可在配置缩小min_us/max_us。
  关闭时保持最后目标，避免自动回中造成额外动作。
- 启动和连接只查询控制板电压，不发送默认舵机位置；8路按住按钮、9路拖动才发送。
- 此PWM板没有舵机磁编码器/实际角度回读命令，UI显示目标值及发送结果，不冒充实测位置。
- 电源或USB断开时软件无法保证停止，应使用物理断电。

## 配置
`/home/potato/Slicer_Workspace2/Test/pwm_servo_control/config.json`

`channel8`：中位、正反向端点、方向、按住转速比例hold_speed_percent。
`channel9`：脉宽范围、标称角度范围、方向。
`move_time_ms`：运动时间，默认300ms；8路松开/退出的停止命令固定0ms，直接发中位。
修改后重启本UI。8/9是控制板丝印S8/S9，直接发ID8/9，不是数组索引。
不接触底盘、导航、其它舵机及老工作空间。

## 协议与设备确认
本机USB设备`0483:5750`，manufacturer=MindMotion SOC Solutions，product=Hiwonder，serial=MM32F103RB。
当前为`/dev/hidraw1`，重插后节点可能变化，程序每次按身份查找。
两个CH343串口不是此控制板，程序不会打开它们。

厂家串口协议9600、8N1，帧为`55 55 Length Cmd Parameters`，无校验；命令3控制PWM。
单路：`55 55 08 03 01 time_lo time_hi channel pulse_lo pulse_hi`。
USB实测沿用相同帧，补齐64字节，Linux hidraw写入时前加0号report ID，共65字节。
只读电压查询`55 55 02 0F`返回`55 55 04 0F lo hi`；读取只取有效长度，忽略HID尾部残留。
未使用动作组停止命令冒充连续舵机停止；停止8路通过发送其中位PWM实现。
本地原版协议/说明书及提取文本保存在reference/。
官方参考：https://docs.hiwonder.com/projects/16-Channel-Servo-Controller/en/latest/docs/2_Communication_Protocol.html

## 检查
```bash
./Test/run_pwm_servos.sh --check        # 实板只读电压，不运动
cd Test/pwm_servo_control
python3 -B -m unittest -v test_protocol.py
./run.sh --self-test                   # 真实Tk窗口、演示数据、无硬件
./run.sh --demo                        # 手动试界面，完全不连接板子
```
正常退出应关闭窗口；不要用kill -9结束旋转中的进程。
