# 2026-11-22 多仓库与车端工作区整理

## 整合顺序

首 WGS84 自动配准先完成 [PR #2](https://github.com/20DCYy06/slicer-ugv/pull/2) 并按必要 `prepare` 检查合入 `86c36250c975e58b7a0e24b92dff1e1b72c3d400`。新布局从该主线独立分支实施。原功能分支完整历史在新主检出 `archive/rtk-auto-registration-local` 和任务 bundle 保留。功能附件和验证日志移入独立开发记录。[自动配准交接](RTK_AUTO_REGISTRATION.md)

FAST-LIO 和 Livox ROS Driver2 已在团队 fork 保留原上游历史、车端原树快照和必要路径适配，并通过全新匿名检出验证。SDK2 使用固定上游版本。硬件仓发布和最终固定 SHA 随本交接阶段记录更新。

## 目录与实际运行

`~/workspace/slicer/main/src/slicer-ugv` 是独立 Git 主检出。迁移任务检出在 `dev/repository-layout/src/slicer-ugv`，与原现场 Git 目录无依赖。相同结构用于开发和发布。固定依赖使用 `W/src/third_party`、`W/deps`，构建和安装在 `W/{build,devel,install}`，运行配置在 `W/runtime`，SDK 安装在 `W/sdk/install`。

旧现场 shell 先 source Noetic，再通过 `~/slicer_noetic_current/install/setup.bash` 选择 v1.7 增量安装。审计时所选 RTK、NoRTK、Livox、FAST-LIO 和集成空地包均来自该安装，本机默认 ROS Master 无响应。旧入口脚本仍可从旧车端检出读到源码 Python 或 launch。旧源码检出 NoRTK 的 `max_reverse=0.3`，已部署生成配置与共同主线为 `0.5`。新生成结果维持 `0.5`，以实际部署和原恢复记录为依据。旧增量安装的统一整车及依赖构建 SHA 未知。源检出版本不能代填其安装版本。

`~/workspace/slicer/releases/<release-id>` 保存独立构建的候选，正式选择以 `current` 为准。日志、状态和采集分别在根目录的 `logs/`、`site/state/` 和 `data/`。外部 GNSS、凭据及相机标定仍按现场配置。载入、停止、切换与回退见[操作手册](../OPERATIONS.md)和[维护说明](../MAINTENANCE.md)。

## Git、V2 与验证

已审计并清理 28 条路径确实不存在的旧 worktree 元数据，保留分支、原活跃功能 worktree 和可恢复提交。无真实工作目录或分支被删除。审计证据在 `dev/repository-layout/logs/`。

独立 AG_Client 工作区保持飞机团队维护。其当前 typed V2 与整车集成 V2 JSON 的同名话题类型不同。整车工作区只获取车载集成的 `ag_uav_client`、`ag_ugv_task`、`slicer_air_ground_msgs`。已核对来源、消息 MD5、WGS84 协议与队列时序。[V2 联调边界](../../src/ag_uav_client/docs/protocol_v2.md)

组合源码及生成配置的静态验证和现有离线测试已执行。干净候选构建、远端 CI、隔离 ROS 与现场验收分别记录。
