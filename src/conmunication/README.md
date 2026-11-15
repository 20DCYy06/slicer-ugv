# Workspace2 通信（目录名按要求保留conmunication）

原`Slicer_Workspace/communication`已独立迁入本目录。旧树、旧运行中的ugv容器均不修改/重启。不启动底盘、导航、巡航或目标。没有给车内导航全量设置ROS_NAMESPACE。

## 数据接口

|车内源|对外话题|类型|频率|
|---|---|---|---|
|/odom|/ugv/odom|nav_msgs/Odometry|最高5Hz，保留完整消息|
|/global/odom|/ugv/pose|geometry_msgs/PoseWithCovarianceStamped|最高1Hz，原header+pose字段提取|
|/slicer_c30d_driver/ready|/ugv/chassis_ready|std_msgs/Bool|变化时发布、锁存|
|/slicer_multi_goal_patrol/state|/ugv/mission/state|std_msgs/String|旧纯SLAM巡航状态；变化时发布、锁存|

正式 RTK 空地任务优先让电脑/飞机直接连接车端同一 Master。现场若继续使用已有 FKIE 双 Master 通信链，可直接同步下面两个话题，不经 relay 改名：

|话题|类型|发布端与锁存|
|---|---|---|
|/air_ground/command|slicer_air_ground_msgs/Command|电脑/飞机侧唯一任务出口；不锁存|
|/air_ground/state|std_msgs/String|车端任务节点；锁存最新状态并周期更新|

任务协议、车端启动和电脑命令见[空地任务接口 V1](../Online_Navigation_with_RTK/docs/air_ground_interface_v1.md)。`/ugv/mission/state` 只保留旧巡航含义，不能作为正式任务状态。FKIE 的 `sync_topics` 只导入原名注册，不做消息类型转换，也不改变发布端锁存属性；`config/sync_topics.yaml` 中 `air_ground_interfaces` 仅记录公开类型与锁存约定。

所有前缀小写。RTK位姿只做消息类型字段适配：时间戳、Header.seq、frame_id、位姿和协方差保留，不查询TF、不换坐标、不添加滤波。连续里程计仍在odom坐标系，RTK全局位姿在map/ENU坐标系；同名map不自动代表与UAV共享原点。

没有/global/odom时/ugv/pose不发布伪数据；无AMCL的纯SLAM导航不会凭空得到该RTK位姿。RTK任务不发布旧slicer巡航state时，旧巡航出口也不会伪造消息。离散锁存状态是最后一次观察值，不代表心跳。没有/cmd_vel、/tf、/tf_static外发或速度输入通道，没有/ugv/test外发。正式任务指令只进入 `/air_ground/command`。

## 命令

```bash
cd /home/potato/Slicer_Workspace2/src/conmunication
./run.sh build
./run.sh check
./run.sh status
# 仅静态配置解析，不启动节点或连接ROS：
python3 -B test_air_ground_config.py -v
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
  - /air_ground/command
  - /air_ground/state
  - /ugv/odom
  - /ugv/pose
  - /ugv/chassis_ready
  - /ugv/mission/state
```
飞机端还需要允许 `/air_ground/command` 与 `/air_ground/state` 的同步，和车端使用同一消息包版本；本机白名单保留/uav/test接收。飞机端由队友在自己的工程内重载通信进程。本次没有UAV SSH凭据，不提供猜测路径的重启命令。

在车端 RTK 工程执行 `./run.sh build` 会生成并安装 `slicer_air_ground_msgs`。电脑/飞机端将该工程的 `src/slicer_air_ground_msgs` 最小包放入其 Noetic Catkin 工作区 `src/`，执行 `catkin_make` 后 `source devel/setup.bash`；发布正式指令的进程、车端任务节点及两端通信环境都须能解析同一版消息包。FKIE 只同步注册，relay 继续只处理原有标准消息；这不代替消息包构建和 source。

本目录 `run.sh` 只在同级 `../Online_Navigation_with_RTK/install/setup.bash` 存在时，将我方这份 install 只读挂到 `/rtk_install`，供新建 sync/discovery/relay 容器在 source Noetic 后执行 `source /rtk_install/setup.bash --extend`。不自动构建、不挂载队友或旧工程。没有 install 时仍可运行原标准消息 relay 和隔离 `check`，但正式任务接入前须先完成消息包构建。飞机侧也需在其消息发布及通信进程环境中构建/source 相同定义。

已有通信容器的挂载不会由 `start`、`reload-sync` 自动更新。要应用新消息安装挂载，须按实际实例清单明确停止并重建自己的 `workspace2-ugv-` 新前缀容器；不要修改仍挂载旧工程的 `ugv-*` 容器，也不能仅重启旧容器就报告正式任务通信环境已更新。

## 旧UAV平面路点：默认关闭，只存储，不自动导航

`launch/fkie_ugv_sync.launch` 默认只启动 FKIE 同步，`enable_legacy_waypoints:=false` 不启动旧文件保存节点，默认同步名单也不导入 `/uav/mission/waypoints`。`config/fkie_ugv_sync.launch` 保留同样默认值的新目录内兼容入口。正式任务运行时不要同时运行旧 `patrol` 或其他向同一导航 Action 发目标的客户端。

确需旧纯SLAM流程时，显式启用 `enable_legacy_waypoints:=true`，并单独将 `/uav/mission/waypoints` 加回两端同步名单。通过本脚本创建新的 sync 容器可使用 `UGV_ENABLE_LEGACY_WAYPOINTS=true ./run.sh start`；只有显式启用时才把旧路点目录挂载为可写。已存在容器的启动参数不会被 `start` 或 `reload-sync` 改写，改变此开关需要用户停止并重建该 sync 容器，不能只重启。正式任务模式应使用默认 `false` 创建。

旧节点接收 `/uav/mission/waypoints` 的 std_msgs/String JSON 批次，保留原校验、文件锁、原子替换和备份逻辑。

默认写入新纯SLAM导航：
`../Online_Navigation_Without_RTK/catkin_ws/src/slicer_navigation_without_mapping/config/waypoints.yaml`。

```json
{"frame_id":"map","waypoints":[{"name":"point_001","x":1.0,"y":0.2,"z":0.0,"yaw":0.0,"wait_s":2.0}]}
```

单位米/弧度，坐标必须属于双方约定的同一次map；这里不是经纬度。用户之后人工启动纯SLAM包patrol。正式 RTK WGS84 任务使用 `/air_ground/command`；原 geographic.py capture/plan/send JSON 工具保留，不将其目标文件误当作这个平面 YAML。可用UGV_WAYPOINT_DIR改变保存目录，切换路径后需要重建新sync容器挂载，不仅重启旧容器。

verify_relay.py只在network=none、无设备、私有Master内发合成状态/旧路点，检查限频、字段一致性、状态锁存与只写文件；这不验证正式任务执行。`test_air_ground_config.py` 解析 XML/YAML，并在临时目录用假 Docker/假 master 探针检查生成的容器参数：正式类型约定、同步名单、旧接收器默认关闭、原状态接口、消息安装只读挂载/source 顺序，以及无 install 时 `check` 仍为 network=none；测试不调用真实 Docker 或网络。生产start不会启动任何测试发布器。历史交接说明保存在docs/reference，旧路径命令仅供参考；当前入口以本README为准。

跨机使用时，建议先完成旧实例交接，再由通信启动共享Master，导航终端使用相同网络变量：
```bash
export ROS_MASTER_URI=http://192.168.3.68:11311
export ROS_IP=192.168.3.68
```
这只决定网络地址，不改变车内话题名或全局namespace。换网络时同步更改这两项及network.env，并确认UAV可回连；实际IP不能照搬历史地址。

本次交付验证见`migration/validation.json`；参考来源与原文件不变核验见`migration/reference_manifest.json`和`migration/source_isolation_report.json`。实际跨电台和户外行驶未验证，未启停现有实车/通信服务。
