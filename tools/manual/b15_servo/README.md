# PB14迁移（v1.4已刷写、只读查询与四轮零输出验收通过）

新入口：`./tools/manual/run_b14_servo.sh --run`；旧run_b15_servo.sh入口保持兼容。
融合入口仍为`./tools/manual/run_combined_gamepad.sh --run`。
启动时查询并打印实际引脚。PB14固件回复B14P/pin14，旧固件回复B15P/pin15。
若打印PB15，说明板子尚未刷入v1.4，修改上位机无法改变物理输出引脚。
配置脉宽保持原值，A/Y对应2400us，D/A对应600us，中位1500us。
新固件路径：Hardware/Firmware/v1.4/bin/firmware.bin。
接线时先断开舵机电源，再将信号线由PB15移至PB14，电源和地线不变。

以下为原接口与协议说明，PB15/TIM12_CH2在v1.4中替换为PB14/TIM12_CH1：

# C30D PB15连续旋转舵机

启动：`./tools/manual/run_b15_servo.sh --run`
A顺时针、D逆时针点动，长按由终端重复继续；空格停，Q退出。SSH不能精确读取松键，每次点动0.3s。
只读查询：`./tools/manual/run_b15_servo.sh --check`；离线：`./tools/manual/run_b15_servo.sh --self-test`。
配置：tools/manual/b15_servo/config.json，方向反了交换clockwise_us与counterclockwise_us。
实际顺逆与舵机型号及观察方向有关，未做实物运动确认。

PB15属于STM32主板TIM12_CH2，不属于独立Hiwonder板S8/S9。
厂家资源表reference/resources.pdf第1页确认其为舵机接口；定时器采用AF9，50Hz，脉宽单位us。
原v1.1没有此接口；配套v1.2已刷入并通过整片校验、四路零输出及PB15只读验收，路径Hardware/Firmware/v1.2/bin/firmware.bin。
当前板若无此接口，程序报错并退出，不会盲发其它协议。

候选协议：沿用SLC2 CRC帧，0x30空载荷查询→0x86返回`<4sBBHH>` B15P/1/15/1500/500；
0x31载荷`<H>`脉宽500–2500→0x87回显脉宽。回显只表示寄存器已设置，不是舵机编码器位置反馈。
第一次显式控制才开启PB15；500ms无刷新回到1500us。断USB后固件仍有此回中机制。
PWM连续舵机无位置读取，1500us是否真正停止仍需现场确认。电源接法按舵机额定电压，信号与主板共地。

独占C30D串口，与导航、WASD或手柄不能同时直接占用。程序从不发送车轮速度、PWM或故障解锁指令。
不自动刷写：需动力12V断开、Motor OFF、仅USB且重新确认后再刷，当前板可能已有他人更新。
