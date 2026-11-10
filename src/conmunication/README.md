# Workspace2 通信（目录名按要求保留conmunication）

原`Slicer_Workspace/communication`已独立迁入本目录。旧树、旧运行中的ugv容器均不修改/重启。不启动底盘、导航、巡航或目标。没有给车内导航全量设置ROS_NAMESPACE。

## 数据接口

|车内源|对外话题|类型|频率|
|---|---|---|---|
|/odom|/ugv/odom|nav_msgs/Odometry|最高5Hz，保留完整消息|
|/global/odom|/ugv/pose|geometry_msgs/PoseWithCovarianceStamped|最高1Hz，原header+pose字段提取|
|/slicer_c30d_driver/ready|/ugv/chassis_ready|std_msgs/Bool|变化时发布、锁存|
|/slicer_multi_goal_patrol/state|/ugv/mission/state|std_msgs/String|变化时发布、锁存|

所有前缀小写。RTK位姿只做消息类型字段适配：时间戳、Header.seq、frame_id、位姿和协方差保留，不查询TF、不换坐标、不添加滤波。连续里程计仍在odom坐标系，RTK全局位姿在map/ENU坐标系；同名map不自动代表与UAV共享原点。

没有/global/odom时/ugv/pose不发布伪数据；无AMCL的纯SLAM导航不会凭空得到该RTK位姿。RTK原任务接口不发布slicer巡航state时，相应任务状态出口也不会伪造消息。离散锁存状态是最后一次观察值，不代表心跳。没有/cmd_vel、/tf、/tf_static外发或输入控制通道，没有/ugv/test外发。

## 命令

```bash
cd /home/potato/Slicer_Workspace2/src/conmunication
./run.sh build
./run.sh check
./run.sh status
# 确认网络与旧实例交接后，由用户执行：
./run.sh start
./run.sh reload-sync
./run.sh stop
```

专用镜像`workspace2-communication:local`继承已验证的`workspace2-online-navigation:local`，仅确认FKIE依赖，不覆盖导航镜像标签。

新容器前缀为`workspace2-ugv-`，不会悄悄复用仍挂载旧源码的`ugv-*`容器。正式切换前应由用户停止旧`ugv-state-relay/ugv-sync/ugv-discovery`，避免同一ROS Master上有两套同名节点；不要为切换通信停止正在被导航使用的Master。stop只停止新前缀的relay/sync/discovery，保留共享Master。

## 网络配置（切换前必须核对）

`config/network.env`：当前UGV默认192.168.3.68，UAV地址沿用旧记录192.168.10.32，**后者尚未现场确认**。两台机载电脑必须路由可达，ROS_IP使用电脑网卡地址，不能用电台管理IP。可通过环境变量覆盖UGV_ROS_IP/UAV_ROS_IP/ROS_MASTER_URI。导航和通信必须连接同一UGV Master。

当前未操作UAV，也未测试实际跨电台延迟。飞机端FKIE接收白名单请使用：
```yaml
sync_topics:
  - /ugv/odom
  - /ugv/pose
  - /ugv/chassis_ready
  - /ugv/mission/state
```
飞机端还需要允许发布`/uav/mission/waypoints`供UGV导入；本机白名单保留/uav/test接收。飞机端由队友在自己的工程内重载通信进程。本次没有UAV SSH凭据，不提供猜测路径的重启命令。

## UAV路点：只存储，不自动导航

`launch/fkie_ugv_sync.launch`同时启动FKIE同步和保存节点；`config/fkie_ugv_sync.launch`保留新目录内兼容入口。接收`/uav/mission/waypoints`的std_msgs/String JSON批次，使用原校验、文件锁、原子替换和备份逻辑。

默认写入新纯SLAM导航：
`../Online_Navigation_Without_RTK/catkin_ws/src/slicer_navigation_without_mapping/config/waypoints.yaml`。

```json
{"frame_id":"map","waypoints":[{"name":"point_001","x":1.0,"y":0.2,"z":0.0,"yaw":0.0,"wait_s":2.0}]}
```

单位米/弧度，坐标必须属于双方约定的同一次map；这里不是经纬度。用户之后人工启动纯SLAM包patrol。本RTK工程的WGS84目标仍由原geographic.py capture/plan/send JSON接口管理，不将其误当作这个平面YAML。可用UGV_WAYPOINT_DIR改变保存目录，切换路径后需要重建新sync容器挂载，不仅重启旧容器。

verify_relay.py只在network=none、无设备、私有Master内发合成状态/路点，检查限频、字段一致性、状态锁存与只写文件。生产start不会启动任何测试发布器。历史交接说明保存在docs/reference，旧路径命令仅供参考；当前入口以本README为准。

跨机使用时，建议先完成旧实例交接，再由通信启动共享Master，导航终端使用相同网络变量：
```bash
export ROS_MASTER_URI=http://192.168.3.68:11311
export ROS_IP=192.168.3.68
```
这只决定网络地址，不改变车内话题名或全局namespace。换网络时同步更改这两项及network.env，并确认UAV可回连；实际IP不能照搬历史地址。

本次交付验证见`migration/validation.json`；参考来源与原文件不变核验见`migration/reference_manifest.json`和`migration/source_isolation_report.json`。实际跨电台和户外行驶未验证，未启停现有实车/通信服务。
