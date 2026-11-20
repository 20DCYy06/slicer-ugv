# 新底盘户外RTK操作单 · 2026-11-11

当前使用我方独立新工程；队友树只作依赖，不修改。DWA + 0x16 + LIO/轮速EKF + 全局robot_localization + Guard/Mux/Watchdog；普通导航默认 `visual_reverse=false`，不启动视觉伺服、深度相机或Gmapping。昨日视觉对齐成功不代表RTK导航通过。

**加测02已结束，当前收车取包，不再继续初始化。** 雷达正上方26cm固定安装已取得静态Fixed，初始化、短返回及前6m/右6m/返回标记整组完成；软件停车后地理复核通过，用户长返回粗估距标记约10cm，非独立尺测精度。详见[02结果报告](RTK_FIELD_RESULT_20261111_02.md)。下列命令保留供后续使用，目录统一为 `evidence/rtk-20261111-02`；已有结果不可覆盖，下一轮须换新编号。01段原录包及收车快照继续保留。

**1．每个车端 potato 宿主终端先执行这三行。** IP变化时统一更新所有终端，不能混入旧电台地址。

```bash
cd /home/potato/slicer_workspace2_codex/src/Online_Navigation_with_RTK
export ROS_MASTER_URI=http://192.168.43.187:11311 ROS_IP=192.168.43.187
export UGV_SLICER_ROOT=/home/potato/Slicer_Workspace2/src/Online_Navigation_Without_RTK
```

定位/GNSS已由助手通过 `slicer-rtk-sensors` 启动，当前不要重复启动。重启后恢复才使用：

```bash
UGV_GNSS_DEVICE=/dev/ttyCH343USB0 ./run.sh sensors
```

该GNSS端口本次已核对USB序列号 `5185028488`，重新插拔后需核对；不要误用底盘端口。账号位于车端受保护的 `/home/potato/.config/slicer/ntrip.json`，容器只读挂载，不把密码写入源码或结果包。服务采用WGS84端口8002、RTCM33GRCEJpro；此前认证收到200成功响应，差分已实际回注，未发送虚构GGA。室内初查q=0、GGA约1Hz是历史记录；当前26cm固定安装位的持续Fixed及有效输出频率仍以本加测段实时检查为准。

**2．启动底盘、odom导航与本机RViz；先保持停车。** 退出昨天的视觉控制窗口/控制容器及其他导航、WASD/手柄，避免重复串口或速度发布者。底盘由用户在独立终端执行，按新板 `5CF0016129` 自动识别：

```bash
./run.sh driver
```

本轮底盘已由用户启动，助手已启动 `slicer-rtk-navigation`（odom档、Mux startup held），不要重复。以后没有该实例时，导航终端恢复命令为：

```bash
./run.sh nav-odom
```

Jetson本机显示终端（须已登录Jetson桌面；关闭窗口即可结束显示）：

```bash
UGV_GUI=1 ./run.sh shell rviz -d /workspace/install/share/ugv_bringup/rviz/odom_navigation.rviz
```

操作终端进入shell，后文 `rosrun` 都在此 cup shell执行：

```bash
./run.sh shell
rosrun ugv_slicer_control control.py state
rosrun cup_gnss gnss_status.py --once --json
rosrun ugv_navigation geographic.py status
mkdir -p evidence/rtk-20261111-02/targets
```

户外等待 `gga_quality=4`、`fresh_rtk_usable=true` 和有效RTCM回注；初始化前 `registered=false` 正常。核对LIO实时、scan/障碍与现场对应、底盘ready且Mux held。**加测02使用雷达正上方26cm的固定安装**：`gnss_site.yaml` 的水平杆臂已在source/install同步为 `[0.065,0]m`，z仍null。26cm是相对雷达的垂直高度，不是实测base_link三维天线高度；二维补偿仍只做一次，不沿用前一车尾位置的 `[-0.135,0]`。最终尺测参考点为 **base_link原点的地面垂直投影**。从室内搬到户外若抬起过车，放稳后先由助手重新核对/初始化LIO，不直接沿用室内匹配结果。

历史01段：原安装附近UM982报告强CW干扰；同一笔记本USB供电时，移开/放回原位的干扰报告随位置变化。抬高手持接回Jetson时曾达到q4、18星、fresh_RTK=true；后来雷达后约20cm的车尾安装45秒仍全程q2，Fixed复核未通过。**这些都不是当前26cm固定安装的验收，也不是运动通过**；本段结果另录。雷达断电对照后LIO曾停滞，当前已恢复定位链，继续核对新鲜时间戳。

另开采集终端，先执行第1节，再运行；等录包启动后才运动：

```bash
./run.sh shell python3 tools/collect_odom_test.py --navigation --duration 900 --output evidence/rtk-20261111-02/capture
```

包含GNSS原始定位/质量/诊断、全局及局部odom、TF、Action、global/local plan、Costmap、scan、速度链和已有MCU反馈。没有采到的项目保留为缺口。目录已存在时换新编号，不覆盖。

**3．初始化包含前进约2.5m；现场确认后由用户执行。** 人员在旁，前方可达、A轮无碰擦、急停和接管可用：

```bash
rosrun ugv_slicer_control control.py release && rosrun ugv_navigation geographic.py initialize --distance 2.5 --output evidence/rtk-20261111-02/alignment.json; rosrun ugv_slicer_control control.py hold
```

只在 `registration.success=true` 后继续。保持定位/GNSS/底盘运行，仅停止当前odom导航：本轮由助手切换；用户独立操作时对本轮后台实例先执行下面的 `docker stop`，若改为前台 `run.sh nav-odom` 则仅在该终端Ctrl-C。然后切map导航，不能与旧实例并行：

```bash
docker stop --time 10 slicer-rtk-navigation
./run.sh nav
```

关闭旧odom RViz，在显示终端打开map视图：

```bash
./run.sh rviz
```

map全局Costmap为40×40m、0.05m，原点周围x/y均 `[-20,20)`；局部4×4m滚动。范围不代表已观测为空地。沿用DWA线/角速度上限0.4m/s、0.25rad/s，实际能否跟随以本轮反馈为准，不新增限速。

**4．停车采点，然后从新地点返回。** 将同一base_link地面参考点停在标记上，保持hold：

```bash
rosrun ugv_navigation geographic.py capture --duration 3 --output evidence/rtk-20261111-02/targets/mark01.json
```

capture只采新数据并补偿一次水平杆臂，不初始化、不运动。输出是二维水平base_link目标，保留天线Up约定，不能当实测车中心三维高度。随后按现场既有安全方式驶离约3～5m，保持定位；不要抬搬后假定LIO连续。抬搬、重启或障碍错位时，停车核查定位，必要时重新初始化；WGS84目标文件可以沿用。

```bash
rosrun ugv_navigation geographic.py plan --target-file evidence/rtk-20261111-02/targets/mark01.json --output evidence/rtk-20261111-02/mark01-plan.json
```

plan不运动。RViz核对目标、路径和真实通道，确认人员在旁及接管条件后由用户执行：

```bash
rosrun ugv_slicer_control control.py release && rosrun ugv_navigation geographic.py send --target-file evidence/rtk-20261111-02/targets/mark01.json --timeout 120 --output evidence/rtk-20261111-02/mark01-return.json; rosrun ugv_slicer_control control.py hold
```

省略yaw采用接近目标方位，不强制地图零航向。分别记录Action结果、停车窗口及停车后RTK地理残差；软件地理复核不是独立测量真值。结束至少5秒不挪车，量base_link参考点到标记的水平距离，说明碰擦、急停或其他干预。若RTK降级，当前实现不会仅因此自动取消运行目标，应人工停车。

**5．停止与收集。** 操作shell：

```bash
rosrun ugv_navigation goal.py cancel
rosrun ugv_slicer_control control.py hold
```

异常立即物理急停/接管，不自动清锁重试。采集终端Ctrl-C只结束录包，等待正常关闭；带回整个 `evidence/rtk-20261111-02` 及尺测/观察记录，保留已归档01段。停用RViz可直接关窗口。本次两包已正常关闭并取回本地校验，后台定位/导航已停止；已完成与未证明项统一以[02结果报告](RTK_FIELD_RESULT_20261111_02.md)为准。独立尺测、S形轨迹量化、绕障及不同路面重复稳定性尚待后续，搬远重新初始化返回没有执行；不以收车后的GNSS/定位变化改写本次已完成结果。
