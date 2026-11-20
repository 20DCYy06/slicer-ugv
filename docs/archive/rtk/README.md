# Workspace2 RTK导航迁移版

来源：`/home/potato/slicer_ugv_ros1`。保留原GNSS/NTRIP单串口、ENU配准、robot_localization全局EKF、DWA、Guard/Mux、地理采点/预规划/返回逻辑。源码、配置与构建安装都在当前新目录；不再挂载/source旧Slicer_Workspace或旧slicer_ugv_ros1的devel/install。

## 运行结构

新纯SLAM工程提供只读的雷达/FAST-LIO/slicer_bringup底层构建环境，RTK项目自行生成online_runtime传感器配置。局部EKF独占odom→base_link；RTK全局EKF独占map→odom。默认不启动Gmapping、map_server、AMCL、深度相机或RTK以外的定位切换。

RTK模式中的“图”是实时雷达障碍缓存：global为map/ENU下40×40m固定costmap，local为odom下4×4m滚动costmap；它不是持久化Gmapping占据地图。保留原显式mapping候选分支，但不能与RTK全局EKF同时启动来争抢TF。

## 构建与检查

```bash
cd /home/potato/Slicer_Workspace2/src/Online_Navigation_with_RTK
./run.sh build
./run.sh check
```

基础镜像使用本机已有`ugv-online-rtk:arm64-20261030`，包含成熟nmea_navsat_driver/ntrip_client、DWA、robot_localization、Livox SDK等依赖。NoRTK项目必须已build一次。`check`严格network=none、无设备、独立Master，模拟GNSS/Action/TF不等于实车效果。

## 新底盘与几何

- 实际串口身份：1a86:55d4，序列号5CF0016129；不再使用旧6377板。
- PWM符号[1,-1,-1,-1]，编码器符号[1,-1,1,-1]，来自Workspace2/Hardware/config.yaml。
- 只从body_speed入口发0x16的线速度/角速度；0x18确认80%PWM硬上限，百分比100对应1280/1600 ticks。不把Twist改成开环PWM。
- 轮径65mm、轮中心距179mm、轴距130mm；雷达前65mm、高95mm。扫描最低高度相对base_link地面为50mm。
- 天线已确认在雷达上方，二维水平杆臂[0.065,0]m，高度仍待实测且不参与本二维补偿。
- footprint由新纯SLAM工程同一几何生成，不继续套旧更大前伸包络；不重复加padding。
- DWA沿用原0.4m/s、0.25rad/s上限和既有权重/容差，没有为迁移新加运动限制。原Mux启动held，用户显式release才接通导航。

本次未刷写固件。v1.1 MCU内部前馈几何仍是0.15m轮距/0.17m轴距，新实测几何已用于上位机里程计/配置；如需下位机精确同步需另行编译并按断动力电流程刷写。

## 用户现场顺序（下列driver、release、initialize、send涉及真实硬件/运动）

先退出Test的WASD/手柄与另一套导航，避免重复串口、雷达和TF。各终端都在当前目录，连接相同ROS Master。默认Master为本机127.0.0.1:11311；改网络只修改`src/ugv_bringup/config/slicer_runtime.env`或对应环境变量。

1. 定位终端：接收机USB序列号沿用5185028488，先核对设备实际存在，显式传入，不回退到底盘串口。
```bash
UGV_GNSS_DEVICE=/dev/serial/by-id/usb-1a86_USB_Single_Serial_5185028488-if00 ./run.sh sensors
```
NTRIP凭据只读引用`/home/potato/.config/slicer/ntrip.json`或UGV_NTRIP_CREDENTIALS指定路径；本次没有读取/复制密码，旧临时账户是否有效需户外核验。源文件`src/ugv_bringup/config/slicer_gnss.yaml`含接收机、波特率与差分服务设置；不是已验证当前账号可用的承诺。

2. 底盘终端（由用户启动，自动按新板身份查找串口）：
```bash
./run.sh driver
```
3. 导航终端，先odom起步档（没有预配准map TF也能建立导航）：
```bash
./run.sh nav-odom
```
4. 操作终端，只读状态：
```bash
./run.sh shell rosrun ugv_slicer_control control.py state
./run.sh shell rosrun cup_gnss gnss_status.py --once --json
./run.sh shell rosrun ugv_navigation geographic.py status
```
5. 户外RTK Fixed、新鲜数据且前方有空间时，用户明确执行初始化；**该命令会前进约2.5米**，不是静态配置。
```bash
./run.sh shell rosrun ugv_slicer_control control.py release
./run.sh shell rosrun ugv_navigation geographic.py initialize --distance 2.5 --output /workspace/data/alignment.json
./run.sh shell rosrun ugv_slicer_control control.py hold
```
6. 仅Ctrl-C停止导航终端的nav-odom，保持定位/GNSS/底盘运行，然后切RTK map导航：
```bash
./run.sh nav
./run.sh rviz
```
7. 车体base_link地面投影停在标记上，保持hold采点；然后先预规划，再由用户执行返回：
```bash
./run.sh shell rosrun ugv_navigation geographic.py capture --duration 3 --output /workspace/data/mark01.json
./run.sh shell rosrun ugv_navigation geographic.py plan --target-file /workspace/data/mark01.json --output /workspace/data/mark01-plan.json
# 查看计划、确认现场条件后：
./run.sh shell rosrun ugv_slicer_control control.py release
./run.sh shell rosrun ugv_navigation geographic.py send --target-file /workspace/data/mark01.json --timeout 120 --output /workspace/data/mark01-return.json
./run.sh shell rosrun ugv_slicer_control control.py hold
```

新启动从未配准状态开始，不复制旧会话registered/原点状态。capture/plan不发运动，initialize/send才发Action。需要共享RTK坐标时先与UAV约定WGS84/ENU原点、坐标轴和时间同步；同名map不足以建立共用坐标系。无RTK时不伪造地理到达成功。

停止/接管：先`./run.sh shell rosrun ugv_slicer_control control.py hold`，再Ctrl-C各终端。需要时使用物理急停。hold/取消/目标/初始化全由用户明确执行，本次迁移验证不启动真实driver、不release、不发实车目标。

原详细流程和未验证项保留在`docs/field/OUTDOOR_RTK_20261031.md`，其中旧绝对路径/旧底盘数字属于历史，当前入口/尺寸以本README和本次生成配置为准。历史日志、bag、凭据、旧编译产物和firmware均未复制。

配置生效规则：运行环境source本项目install；修改src/ugv_bringup/config、Hardware身份/极性或基础几何后，执行`./run.sh build`重新生成并安装，再重启对应节点。`prepare`只生成源码配置，不代替install更新。

本次交付验证见`migration/validation.json`；参考来源与原文件不变核验见`migration/reference_manifest.json`和`migration/source_isolation_report.json`。实际跨电台和户外行驶未验证，未启停现有实车/通信服务。
