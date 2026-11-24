# Slicer UGV 共同交接

整车共同主线使用 Focal / ROS1 Noetic。首任务自动 RTK 配准已通过隔离验收，经 [PR #2](https://github.com/20DCYy06/slicer-ugv/pull/2) 合入。多仓布局见 [PR #3](https://github.com/20DCYy06/slicer-ugv/pull/3) 和[多仓交接](docs/handoffs/WORKSPACE_SPLIT_20261122.md)。自动配准接口与隔离证据见[自动配准交接](docs/handoffs/RTK_AUTO_REGISTRATION.md)。

车端入口见 `~/workspace/slicer/START_HERE.md`。日常 `~/workspace/slicer/run` 由 `current` 选择，当前为 `releases/20261123-rtk-fixed-a7a64c7`。
RTK 主线交付见 [RTK 导航优化](docs/handoffs/RTK_NAVIGATION_OPTIMIZATION_20261124.md)。新桌面将前进段配准、地图显示与返程停稳分开，开阔回环仿真的倒车指令峰值为 0.8 m/s。软件检查与实车验收记录见对应交接和[操作手册](docs/OPERATIONS.md)。

- 在现场复测快速返程、停车净空、RTK Fixed 全局纠偏、WGS84 到点与飞机联合任务。
- 补键盘实车、正式抓取工件与看门狗故障锁存恢复。底盘启动若再次超时，保留握手日志。
- 与飞机团队确认集成 V2 JSON 和独立 AG_Client typed V2 的消息归属与互通方案。[V2 协议](src/ag_uav_client/docs/protocol_v2.md)
