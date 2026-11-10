# 户外候选部署与室内准备（2026-10-31）

## 最新：354169a 已部署（本次回退目标为8e78b86）

2026-10-31已在Jetson `/home/potato/slicer_ugv_ros1` 部署停车窗口、GNSS接收时间戳修复及capture/目标文件接口。ARM64真实Catkin构建安装、188项工具+328项Catkin=516项软件测试，以及7个隔离ROS客户端场景通过；正式目录的capture/plan/send、辅助模块、外部传感器程序与有效DWA/Guard前进上限0.4m/s均核对通过。未打开真实GNSS/底盘串口，未启动实车节点、release或运动；户外实测尚未通过。

GNSS/site/runtime配置及凭据文件原样保留，没有新增低速覆盖。当前唯一现场流程见[RTK初始化、采点、返回](OUTDOOR_RTK_20261031.md)。本次部署证据位于车端`.test-results/rtk-deploy-354169a-20261031/`；完整旧源码文件与install已保留。若需要回退，现场先停车、停止自己启动的ROS launches，再在potato宿主终端执行：

```bash
python3 /home/potato/slicer_ugv_ros1/.test-results/rtk-deploy-354169a-20261031/rollback.py
```

该命令仅恢复本次切换前的8e78b86，不启动节点；凭据和原场地设置保持。以下是较早部署过程记录，不作为本次启动操作单。


Jetson正式工程：`/home/potato/slicer_ugv_ros1`，版本记录在`OUTDOOR_VERSION.json`。户外基线`878810d`使用`ugv-online-rtk:arm64-20261030`完成ARM64 Catkin构建/安装；188项工具测试、317项Catkin测试全部通过，0失败/错误/跳过。后续0.4m/s修订再次通过本地完整505项测试、ARM64的20项启动配置测试、安装和有效Guard上限检查。正式目录安装检查和外部LIO/MID360节点可发现性检查通过。所有测试无设备、`--network none`，没有启动实车ROS节点、串口、NTRIP连接或运动。

本地证据：工程内`.test-results/outdoor-deploy-20261031/`。发布包原样配置接受完整测试后，恢复车端已有配置并另行核对最终安装参数。首次混合配置测试中的DWA默认值断言失败已保留：车端基础上限0.4m/s与测试预期0.125m/s不同。

用户随后明确选择0.4m/s：取消field入口默认0.20巡航覆盖，odom起步档改用不含速度项的地图配置；`dwa.yaml`中`max_vel_x`与`max_vel_trans`统一为0.4。Guard有效上限`min(源配置0.9, DWA 0.4, 平移0.4)=0.4m/s`；Mux原值透传，0x16 driver没有另设0.22m/s上限。线加速度0.2m/s²、角速上限0.25rad/s及现有碰撞/新鲜度/Watchdog规则未改。0.20/0.30文件仅保留为显式选择的对照，不会默认加载。

## 已填写的场地配置

- `src/ugv_bringup/config/slicer_gnss.yaml`：启用用户提供的临时差分服务，`103.143.19.54:8002`，`RTCM33GRCEJpro`。按服务方声明选择WGS84、参考历元2005；软件不做历元变换，尚未实测接收机定位结果。
- `src/ugv_bringup/config/gnss_site.yaml`：`datum: WGS84`；原点留空，明确initialize后固定。天线水平外参继续复用已确认的雷达安装x/y（天线在雷达正上方），没有编造新测量值。
- 账号密码仅在宿主机`/home/potato/.config/slicer/ntrip.json`，权限0600。现有wrapper只读挂载到`/run/secrets/slicer_ntrip.json`，无需每次输入环境变量。临时账户过期后更新该文件；不要写进YAML、ROS参数、源码或录包。
- `src/ugv_bringup/config/slicer_runtime.env`保留当前WiFi地址`192.168.3.68`。改用电台时修改其中master/IP为真实可回连地址，各终端保持一致。

## 户外前室内还需做什么

不必重复低速PWM、MCU、相机或整套导航专项。若当前只有Jetson，软件准备已经完成，不启动缺失设备。装回实车后只做一次静态核验：A右后轮无电池/线束碰擦，急停/人工接管可用；启动后保持Mux held，核对静止`/odom`无跳变、雷达与真实障碍对应、足迹覆盖车体。室内无RTK FIX、`registered=false`正常，不在室内做地理配准或发送地理目标。

车端宿主机每个终端先执行：

```bash
cd /home/potato/slicer_ugv_ros1
```

定位终端（未接GNSS时去掉环境变量并在末尾加`start_gnss:=false`）：

```bash
UGV_GNSS_DEVICE=/dev/serial/by-id/usb-1a86_USB_Single_Serial_5185028488-if00 ./tools/slicer_shell.sh roslaunch ugv_bringup slicer_field_sensors.launch
```

底盘终端由现场人员启动：

```bash
UGV_SERIAL_DEVICE=/dev/c30d ./tools/slicer_shell.sh roslaunch ugv_bringup slicer_body_driver.launch
```

导航终端先用odom起步档；此时不要用默认map档等待尚未完成的配准：

```bash
./tools/slicer_shell.sh roslaunch ugv_bringup slicer_field_navigation.launch geographic:=false
```

操作终端进入`./tools/slicer_shell.sh`后查看：

```bash
rosrun ugv_slicer_control control.py state
rosrun cup_gnss gnss_status.py --once --json
rosrun ugv_navigation geographic.py status
```

已配置网络的观测ROS shell中用odom视图；未配准前map视图缺少TF是预期：

```bash
rviz -d "$(rospack find ugv_bringup)/rviz/odom_navigation.rviz"
or
/home/rescue/slicer_ugv_ros1/tools/rviz_wsl.sh navigation 192.168.3.68
```

室外按[户外操作单](OUTDOOR_RTK_20261031.md)执行：取得新鲜RTK FIX→现场确认后2.5m有限起步配准→hold下只切换导航为默认地理档→一个5～10m标记目标→真实落点/停车测量。不要直接照抄示例ENU目标作为当前车头方向。

## 历史878阶段回退（本轮勿用，使用上方新路径）

旧公开源码、配置与完整install保存在车端`.test-results/outdoor-deploy-878810d-20261031/rollback/`，部署逐文件SHA256在同级`deployment-receipt.json`；凭据单独保留，不进入回退材料。现场先停车并停止自己启动的ROS launches，再从宿主机执行以下命令恢复本次切换前版本（不会自动启动节点）：

```bash
python3 /home/potato/slicer_ugv_ros1/.test-results/outdoor-deploy-878810d-20261031/rollback.py
```

本轮尚未证明：真实GNSS串口及差分服务可用、户外配准精度、连续RTK纠偏、地理目标到点误差、实车安全停车与绕障表现。
