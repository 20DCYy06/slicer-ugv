# AirGrip A/D键盘控制

```bash
cd /home/potato/Slicer_Workspace2
./Test/run_airgrip.sh --run
```
A接通吸取（PUMP_ON）；D/空格断开释放（PUMP_OFF）；Q/Ctrl+C退出。不用回车。
启动仅PING和GET_STATUS，不改变泵/阀状态。每次操作打印设备ACK；ACK不表示物体已吸住。
正常退出时释放本程序可能启动的吸取；物理断线/强制杀死程序不能保证释放，固件不会因通信中断自动释放。
不要与原桌面UI或ROS节点同时占用串口；USB身份锁定MicroPython板，不使用C30D底盘串口。

只读查询：`./Test/run_airgrip.sh --check`，不发送PUMP_ON/PUMP_OFF/SET_LINK。
离线测试：`cd Test/airgrip && python3 -B -m unittest -v test_airgrip.py`。
配置：Test/airgrip/config.json。当前/dev/ttyACM0由by-id路径定位，波特率115200。
依赖Python3、pyserial（已安装）；无需ROS、Docker、编译或新固件。

协议直接逐函数复用官方host/python/host_ros.py：UTF-8文本`@seq|cmd|payload|CRC\n`；CRC16-Modbus。
源分支feature/V1.2T，commit 3eb9371209b9ab602df263db0fac6c9b6f048025。
原说明和驱动保留在reference/，没有导入其中ROS节点。
https://github.com/jiazhuguanhua/UAV_AirGrip/blob/feature/V1.2T/host/ROS1.md

若需要ROS方式，原说明使用`/airgrip_host/set_pump`（std_srvs/SetBool）true/false；本工具与它使用相同底层命令，不同时打开串口。
