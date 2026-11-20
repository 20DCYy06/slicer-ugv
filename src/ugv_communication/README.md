# UGV 通信

`ugv_communication` 保留原 V1 FKIE 白名单及状态 relay。在车端使用已启动的 Noetic master，确认飞机端地址后运行：

```bash
roslaunch ugv_communication fkie_ugv_sync.launch start_discovery:=true start_relay:=true uav_ip:="$UAV_IP"
```

若已有 discovery，可省略 `start_discovery`。若两端共用 master，V1 命令和状态可直接通信。V2 使用自己的同步入口，两套 FKIE 在同一 master 上择一。停止本终端即可结束本次通信 launch。

| 原接口 | 外发接口 |
|---|---|
| `/odom` | `/ugv/odom`（最高 5 Hz） |
| `/global/odom` | `/ugv/pose`（最高 1 Hz，保留时间、坐标系与协方差） |
| `/slicer_c30d_driver/ready` | `/ugv/chassis_ready`（锁存） |
| `/slicer_multi_goal_patrol/state` | `/ugv/mission/state`（旧巡航状态，锁存） |

V1 的 `/air_ground/command` 和 `/air_ground/state` 由 FKIE 保留原名和类型。旧平面路点接收器默认关闭。若确需旧流程，单独为两端加入路点白名单，并显式使用 `enable_legacy_waypoints:=true waypoint_file:=<可写绝对路径>`。该节点只保存路点文件。

旧 Docker 入口和证据位于 [历史记录](../../docs/archive/workspace2/communication/)。现场操作见[整车操作手册](../../docs/OPERATIONS.md)。
