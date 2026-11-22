# BT 行为树开发接入指南交接

2026-11-22，从已 fetch 的 `origin/main` `7b0158e` 建立 `docs/bt-start-here`。检出位于
`~/workspace/slicer/dev/bt-start-here/src/slicer-ugv`。现场选中的 `current` 为
`20261122-workspace-rc1`，整车 SHA 为 `adb4313b0f`。

## 交付

从根和 `src/` 的 README 进入[接入指南](../BT_START_HERE.md)。指南记录两套导航的坐标与完成判据、
三种空地协议的边界、现有视觉进程、底盘交接和无设备测试入口。BT 的接法作为建议供评审。
回退按文档提交处理。

## 验证

- 用 GFM 解析表格、代码块和相对链接，核对路径与标题锚点。用 `bash -n` 检查构建示例，用
  `git diff --check` 检查补丁。
- Focal / Noetic 上，`bash tools/check_offline.sh` 共 287 项通过。视觉 `run.sh --test` 使用固定版本的
  硬件依赖，55 项通过，1 项缺录制帧而跳过。
- 以 RC1 安装和运行参数运行隔离 ROS 用例。首 WGS84 CLI 的预览与接续、整车 V2 命令和暂停/恢复/停止、
  无 RTK 的建图、规划、Guard 和重置均通过。用例使用各自的 loopback Master 与模拟输入。
- 实车证据沿用[RC1 现场记录](RC1_FIELD_ACCEPTANCE_20261122.md)。仍待验收 RTK Fixed 下的自动配准、
  全局纠偏和 WGS84 到点、飞机联调以及完整工件抓放。

## 下一步

审阅本 PR 的接口边界。BT 开工优先确定 V2 协议与任务反馈归属、视觉适配的取消和恢复判据、
以及唯一导航客户端和底盘串口交接时序。
