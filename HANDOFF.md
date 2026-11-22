# Slicer UGV 共同交接

整车共同主线使用 Focal / ROS1 Noetic。首任务自动 RTK 配准已通过隔离验收，经 [PR #2](https://github.com/20DCYy06/slicer-ugv/pull/2) 合入。源码路径调整和现场验收状态见[多仓与 workspace 交接](docs/handoffs/WORKSPACE_SPLIT_20261122.md)和[自动配准交接](docs/handoffs/RTK_AUTO_REGISTRATION.md)。

车端旧增量安装仍由 `~/slicer_noetic_current` 选中，指向 `~/slicer_noetic_c30d_v17`。这套实际安装的统一整车构建 SHA 未知。新工作区位于 `~/workspace/slicer/`。多仓布局见 [PR #3](https://github.com/20DCYy06/slicer-ugv/pull/3)，软件候选为 `releases/20261122-workspace-rc1`。候选及现场切换按[操作手册](docs/OPERATIONS.md)及[维护说明](docs/MAINTENANCE.md)执行。历史恢复组合见[原生恢复基线](docs/handoffs/NATIVE_RESTORED_BASELINE_20261122.md)。

- 车端按现场计划核验候选发布，检查所选入口、安装、配置、模式与回退路径。
- 实车验收自动配准精度、绕障和停车，再进行飞机联合验证。
- 与飞机团队确认集成 V2 JSON 和独立 AG_Client typed V2 的消息归属与互通方案。[V2 协议](src/ag_uav_client/docs/protocol_v2.md)
