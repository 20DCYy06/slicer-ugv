# Slicer UGV 共同交接

整车共同主线使用 Focal / ROS1 Noetic。首任务自动 RTK 配准已通过隔离验收，经 [PR #2](https://github.com/20DCYy06/slicer-ugv/pull/2) 合入。多仓布局见 [PR #3](https://github.com/20DCYy06/slicer-ugv/pull/3) 和[多仓交接](docs/handoffs/WORKSPACE_SPLIT_20261122.md)。自动配准接口与隔离证据见[自动配准交接](docs/handoffs/RTK_AUTO_REGISTRATION.md)。

车端请先打开 `~/workspace/slicer/START_HERE.md`。正式入口为 `~/workspace/slicer/run`，
`current` 选中 `releases/20261122-workspace-rc1`。候选整车提交为
`adb4313b0fb7c825e040fd25b7f1cc61624a26df`。2026-11-22 新工作区接管，实车验证了
传感器、局部定位、NoRTK 短程、Guard/Mux 停车和手柄接管。验收后运行链已停止。
GNSS 串口有数据，Fixed 相关验收待补。旧 v1.7 增量安装与旧源码保留用于回退，旧安装
统一构建 SHA 未知。[现场结果和回退](docs/handoffs/RC1_FIELD_ACCEPTANCE_20261122.md) ·
[操作手册](docs/OPERATIONS.md) · [维护说明](docs/MAINTENANCE.md)

- 恢复 RTK Fixed 后验收自动配准、全局纠偏、WGS84 到点与地理复核，再做飞机联合验证。
- 补键盘实车、正式抓取工件与看门狗故障锁存恢复。底盘启动若再次超时，保留握手日志。
- 与飞机团队确认集成 V2 JSON 和独立 AG_Client typed V2 的消息归属与互通方案。[V2 协议](src/ag_uav_client/docs/protocol_v2.md)
