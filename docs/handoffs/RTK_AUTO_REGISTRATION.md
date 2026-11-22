# 首次 WGS84 任务自动配准

## 代码和工作区

整车 [PR #2](https://github.com/20DCYy06/slicer-ugv/pull/2) 的 `prepare` 通过，合入提交 `86c36250c975e58b7a0e24b92dff1e1b72c3d400`。原本地功能分支的 9 个提交（末端 `919cc3914a2acfb51ed550cb6fa76abb9c5ffff3`）仍可在永久主检出的 `archive/rtk-auto-registration-local` 和任务日志下的 Git bundle 恢复。原验证数据、附件及设备隔离记录归档于 `~/workspace/slicer/dev/rtk-auto-registration/logs/`。已有功能 worktree 保留。新布局续做入口为 `~/workspace/slicer/dev/rtk-auto-registration/src/slicer-ugv` 的 `feat/rtk-auto-registration-field`。新结构及最终验证见[多仓交接](WORKSPACE_SPLIT_20261122.md)。

## 入口与行为

定位和底盘就绪后，通过所选工作区的 `roslaunch ugv_bringup slicer_navigation_runtime.launch` 或桌面启动导航，再发一次 WGS84 目标。V1、V2 和 CLI 使用同一 `/geographic/prepare`，执行器各自保留真实任务。准备中可更换真实目标，暂停保留同一 odom 会话的证据，取消终止任务。静止确认、全局 EKF 输出新鲜和旧实例退出后自动进入 `map`。后续任务复用有效配准。普通 `plan` 只有查询行为。

参数位于 `src/ugv_bringup/config/{navigation_registration,dwa_registration,outdoor_localization}.yaml`，状态位于 `/geographic/navigation_runtime` 和 `/geographic/status`。手动直行配准和模式切换保留在桌面导航诊断。[整车操作手册](../OPERATIONS.md)

## 验证与现场

已有 Focal ARM64 构建、设备隔离 V1/V2 与 CLI 接续、单测及 Qt/RViz 记录。覆盖目标替换、全局输出延迟、切换期间取消、准备暂停恢复与配准复用。移除整车副本的新布局在独立候选中重跑 CLI 与车端 V2 接续，均通过。实际 RTK 精度、绕障、停车距离、现场接管和飞机联调仍需现场完成。V2 航空端接口的当前边界见[集成协议](../../src/ag_uav_client/docs/protocol_v2.md)。
