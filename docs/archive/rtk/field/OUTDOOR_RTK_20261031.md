# 户外 RTK：初始化 → 采点 → 返回标记

本页是当前唯一 RTK 操作单，旧 `OUTDOOR_PREP_20261031.md` 已废止。停车窗口、接收时间戳和采点版本 `354169a` 已于2026-10-31完成ARM64构建并部署到车端独立工程；516项软件测试、7个隔离ROS客户端场景及正式目录安装检查通过。以上为此前软件部署记录，不能替代户外实车验收。以下启动命令和 capture / 文件目标接口已核对；回退到 `8e78b86` 的方法见[部署记录](OUTDOOR_DEPLOYED_20261031.md)。

2026-11-04换轮复测准备：用户报告换用标称65mm轮后粗糙地面转向改善。承接11月1日已刷的`c30d-load-adapt-20261101`（原BIN SHA `1f31fde5…`），本轮只把MCU轮径和我方wheel odom几何改为65mm，保留独立轮限幅、982‰上限、PI、Watchdog和保护。新BIN SHA `1b76a1e64fdd34600fcd7b659b5d4429aa674d82eb1821af4c07e3a956109ebb`，10536字节；烧写与静态核验结果见本轮`evidence/rtk-20261104-01/`。软件/烧写通过不等于新轮RTK导航通过。

10月31日配准和mark01返回属于历史会话：返回因底盘fault未通过，见[记录](MARK01_RETURN_20261031.md)。本轮从新会话开始，不继承旧registered。已有WGS84目标文件仍可读取，但第一次复测先现场新采一个近目标。
本页使用本轮`rtk-20261104-01`目录；已有同名结果时换新编号，保留原文件。目标文件长期保留，返回测试用同一个文件读回。

沿用 DWA + 0x16、LIO/轮速局部EKF与全局robot_localization、Guard/Mux/Watchdog。D455/Gmapping默认关闭；线速度上限**0.4m/s**、角速度上限0.25rad/s、线加速度0.2m/s²；不默认加载低速覆盖。到点容差、停止阈值不改。map全局图40×40m、0.05m，ENU原点周围x/y均[-20,20)；局部图为odom下4×4m滚动。地图范围不等于已观察为空地。

## 1．启动：先 odom 导航，不发目标

Jetson宿主机每个终端先执行：

```bash
cd /home/potato/slicer_ugv_ros1
```

沿用部署的 `src/ugv_bringup/config/slicer_runtime.env` 网络与ARM镜像；各终端同master，不混用旧电台IP和当前局域网IP。接收机、天线和原点仍用现有 `slicer_gnss.yaml` / `gnss_site.yaml`，无需手算经纬度或杆臂。凭据单独只读挂载，临时账户过期需更新宿主机凭据文件。

定位终端（本轮室内已由助手启动；当前不要重复启动。以下供重启后恢复）：

```bash
UGV_GNSS_DEVICE=/dev/serial/by-id/usb-1a86_USB_Single_Serial_5185028488-if00 ./tools/slicer_shell.sh roslaunch ugv_bringup slicer_field_sensors.launch session_file:=/workspace/evidence/rtk-20261104-01/geographic-session.json
```

底盘终端，由现场人员启动：

```bash
UGV_SERIAL_DEVICE=/dev/c30d ./tools/slicer_shell.sh roslaunch ugv_bringup slicer_body_driver.launch
```

导航终端：

```bash
./tools/slicer_shell.sh roslaunch ugv_bringup slicer_field_navigation.launch geographic:=false
```

操作终端：

```bash
./tools/slicer_shell.sh
rosrun ugv_slicer_control control.py state
rosrun cup_gnss gnss_status.py --once --json
rosrun ugv_navigation geographic.py status
mkdir -p evidence/rtk-20261104-01/targets
```

新轮外径65mm，一圈对应约0.204204m；计数仍60000/rev。用户已自行更新`/home/potato/.config/slicer/ntrip.json`。室内GNSS无FIX时可能仍为`waiting_gga`，这不代表认证已经通过。先保持Mux held，确认A轮无碰擦、雷达与实物障碍对应、footprint覆盖突出部分、接管和急停可用。户外要有新鲜RTK Fixed（raw q4且fresh_rtk_usable=true）；初始化前registered=false正常。室内无FIX只做静态检查，不做地理配准。

另开操作shell录同一次完整试验，等RECORDING STARTED；目录须是新目录：

```bash
python3 tools/collect_odom_test.py --navigation --duration 900 --output evidence/rtk-20261104-01/capture
```

## 2．初始化，然后停车切换 map 导航

将车放稳在户外起点后先核对LIO实时更新及障碍对应；若抬搬引起LIO失配，停车下重启定位，再用本轮新会话初始化。留出前方2.5m可达空间，现场确认后执行。**initialize包含一次有限前进Action，会运动**，不是静态配置命令；它不会自动release或重试。

```bash
rosrun ugv_slicer_control control.py release && rosrun ugv_navigation geographic.py initialize --distance 2.5 --output evidence/rtk-20261104-01/alignment.json
rosrun ugv_slicer_control control.py hold
```

确认registration.success=true，保持停车：仅在导航终端Ctrl-C，定位、全局EKF、GNSS和底盘**保持运行**；导航终端重新启动默认map档：

```bash
./tools/slicer_shell.sh roslaunch ugv_bringup slicer_field_navigation.launch
```

观测ROS shell在初始化前用odom视图、成功切换后用map视图：

```bash
rviz -d "$(rospack find ugv_bringup)/rviz/odom_navigation.rviz"
rviz -d "$(rospack find ugv_bringup)/rviz/outdoor_navigation.rviz"
```

以上是二选一的观测命令。确认实际障碍、通道和目标位于图内。不要因旧栅格阻挡就反复清图。明显定位/时间跳变先停车检查；软件有启发式重置检测，但不能保证识别所有搬车/重启。

## 3．停车在标记上采点：不运动

把小车**base_link原点在地面的垂直投影**停到标记上；最终尺测也用这一车体固定参考点，不换成天线、车头或轮轴。保持定位有效并hold：

```bash
rosrun ugv_slicer_control control.py hold
rosrun ugv_navigation geographic.py capture --duration 3 --output evidence/rtk-20261104-01/targets/mark-01.json
```

capture只读取启动后的新数据，采样约3秒（至少沿用1秒静止判据），按同stamp配对原始GNSS/质量，并用对应时刻global姿态和已配置水平杆臂补偿一次。**不初始化、不发Action、不release、不重置定位。** 缺配准、姿态、合格样本或静止窗口时明确失败，不生成可用目标文件；既有文件不覆盖。

保存的WGS84文件包含水平base_link坐标、时间窗、样本数、质量和散布。高度只是保留原GNSS天线Up的二维换算约定，**不是实测三维车中心高度**；散布不是绝对精度。用户无需换算经纬度或杆臂。文件不绑定旧运行generation，重新初始化后仍可读取。

GNSS stamp保留完整消息到达主机时刻并映射到ROS时间，不再用排队后的发布时刻；fix与quality同戳。接收机内部测量/串口传输延迟仍未知，本轮不做PPS授时或延迟标定。

## 4．从别处返回：先 plan，再 send

首轮把返回距离控制在约3～5m，按既有现场流程安全驶离标记，保持定位；不要抬搬后假装里程计连续。如重启定位或配准失效，重做第2步，继续使用同一WGS84文件。

先在操作shell预规划；**capture与plan都不执行运动，send才提交这个目标**。省略yaw时沿用现有“从当前位置指向目标的方位”策略，不暗中要求地图零航向：

```bash
rosrun ugv_navigation geographic.py plan --target-file evidence/rtk-20261104-01/targets/mark-01.json --output evidence/rtk-20261104-01/mark-01-plan.json
```

RViz检查目标、全局路径和地面通道，确认现场接管条件后执行：

```bash
rosrun ugv_slicer_control control.py release && rosrun ugv_navigation geographic.py send --target-file evidence/rtk-20261104-01/targets/mark-01.json --timeout 120 --output evidence/rtk-20261104-01/mark-01-return.json
rosrun ugv_slicer_control control.py hold
```

发送前确认`gnss_status`的`fresh_rtk_usable=true`及`geographic status`的`state=ready`；当前实现不会仅因RTK降级就自动取消已运行目标，应按现场情况人工停止。分别看move_base结果、stop静止窗口、geographic地理残差。只接受**该停车窗口中的新鲜合格RTK**；最多沿用2.5秒预算等新样本，等不到则地理到点未确认。SUCCEEDED、软件静止或较早RTK都不能代替停车后地理复核。

随时停止：

```bash
rosrun ugv_navigation goal.py cancel
rosrun ugv_slicer_control control.py hold
```

需要时物理急停。结果保留Action、停车源时间窗、RTK时刻/质量/残差、干预和base_link到标记的尺测水平误差/停车余程；同源RTK参与融合和复核，不能算独立真值。保留至少5秒静止尾段后结束采集，把`evidence/rtk-20261104-01/capture`、目标文件和alignment/plan/return JSON一并带回。软件验证不能代替下一次户外实车验收。
