# 2026-11-22 多仓库与车端工作区整理

## 整合顺序

首 WGS84 自动配准先完成 [PR #2](https://github.com/20DCYy06/slicer-ugv/pull/2) 并按必要 `prepare` 检查合入 `86c36250c975e58b7a0e24b92dff1e1b72c3d400`。新布局从该主线独立分支实施。原功能分支完整历史在新主检出 `archive/rtk-auto-registration-local` 和任务 bundle 保留。功能附件和验证日志移入独立开发记录。[自动配准交接](RTK_AUTO_REGISTRATION.md)

FAST-LIO 和 Livox ROS Driver2 已在团队 fork 保留原上游历史、车端原树快照和必要路径适配，并通过全新匿名检出验证。SDK2 使用固定上游版本。硬件 [PR #1](https://github.com/20DCYy06/slicer_ugv_hardware/pull/1) 正常合入。主线 `401348db5c59b9f73c2037d7864cd87637b8df42`，固定 pin `a377b45c48c4be91add25b73eacedabdb8c264de`，其 v1.7 镜像、控制配置及 RTK 底层模块已逐字节核对。公开仓省去三份旧 v1.5/v1.6 调试产物，安装台账只保留发布 metadata；完整历史与现场记录另存于硬件任务恢复 bundle。

## 目录与实际运行

`~/workspace/slicer/main/src/slicer-ugv` 是独立 Git 主检出。迁移任务检出在 `dev/repository-layout/src/slicer-ugv`，与原现场 Git 目录无依赖。相同结构用于开发和发布。固定依赖使用 `W/src/third_party`、`W/deps`，构建和安装在 `W/{build,devel,install}`，运行配置在 `W/runtime`，SDK 安装在 `W/sdk/install`。

旧现场 shell 先 source Noetic，再通过 `~/slicer_noetic_current/install/setup.bash` 选择 v1.7 增量安装。审计时所选 RTK、NoRTK、Livox、FAST-LIO 和集成空地包均来自该安装，本机默认 ROS Master 无响应。旧入口脚本仍可从旧车端检出读到源码 Python 或 launch。旧源码检出 NoRTK 的 `max_reverse=0.3`，已部署生成配置与共同主线为 `0.5`。新生成结果维持 `0.5`，以实际部署和原恢复记录为依据。旧增量安装的统一整车及依赖构建 SHA 未知。源检出版本不能代填其安装版本。旧 Livox 驱动二进制可静态核实其 SDK 版本为 1.5.2，新驱动使用当前工作区固定源码编译的同版本库。

`~/workspace/slicer/releases/<release-id>` 保存独立构建的候选，正式选择以 `current` 为准。日志、状态和采集分别在根目录的 `logs/`、`site/state/` 和 `data/`。外部 GNSS、凭据及相机标定仍按现场配置。载入、停止、切换与回退见[操作手册](../OPERATIONS.md)和[维护说明](../MAINTENANCE.md)。

## Git、V2 与验证

已审计并清理 28 条路径确实不存在的旧 worktree 元数据，保留分支、原活跃功能 worktree 和可恢复提交。无真实工作目录或分支被删除。审计证据在 `dev/repository-layout/logs/`。

独立 AG_Client 工作区保持飞机团队维护。其当前 typed V2 与整车集成 V2 JSON 的同名话题类型不同。整车工作区只获取车载集成的 `ag_uav_client`、`ag_ugv_task`、`slicer_air_ground_msgs`。已核对来源、消息 MD5、WGS84 协议与队列时序。[V2 联调边界](../../src/ag_uav_client/docs/protocol_v2.md)

候选 `releases/20261122-workspace-rc1` 已用公开依赖在最终路径完成 Focal ARM64 干净构建与安装。真实解析的 Catkin 包、SDK 头文件及静态库、RTK/NoRTK 生成配置均匹配该工作区。隔离 CLI 的 `plan` 保持无运动，`send` 完成首次接续。车端集成 V2 隔离测试覆盖最新目标、再次任务复用和唯一 TF/Action/速度链。发布入口隔离和拒绝跨工作区参数通过设备隔离测试。现场和飞机仍待验收。


## 固定依赖与本地证据

- FAST-LIO `88c50b57dd931566c547ad1f3a932b792c6d6aea`，Livox ROS Driver2 `470b3988996cd060bf5e4b67b3e834967a56e403`，Livox SDK2 `c0796f04c143143899c87a773d9f6b7136453c0b`。硬件与四个完整 Git URL、相对路径统一在 [dependencies.repos](../../dependencies.repos)。
- Focal ARM64 上已从新布局编译 SDK 和整车。候选的 `release.json` 和 `apt-packages.tsv` 保存选定版本及构建记录，证据在 `logs/20261122-workspace-rc1/`。旧现场 `current` 继续由原增量入口管理，新发布链接等待现场切换。V2 交接证据位于 `dev/offline-ci/logs/`。独立 AG_Client 不加入整车 Catkin 源空间。
- 现场硬件 YAML 可通过 `SLICER_HARDWARE_CONFIG` 选择。发布生成配置和安装资源使用构建时的快照。旧 v1.7 记录来自此前的板端验证，本次发行目录整理不改变固件。


## 公开验收与回退

- 整车布局按 [PR #3](https://github.com/20DCYy06/slicer-ugv/pull/3) 的保护规则集成。软件源以该 PR 和候选恢复分支为准，当前主线 SHA 可用 `git rev-parse origin/main` 核对。
- 干净 Catkin 构建与两项隔离 ROS 场景使用候选应用源码，随后只补发布入口的参数保护与文档，并重新安装和验证入口。远端 `prepare` 用公开固定依赖执行静态与设备隔离单测。
- 现场前先按原版停稳并退出，用发布选择入口核对候选和旧选择。回退同时核对对应源检出与安装。固件与标定单独恢复。
