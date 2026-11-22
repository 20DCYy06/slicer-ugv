# Slicer UGV 项目交接

恢复阶段以 `baseline/native-restored-20261122` 定格。团队确认采用当时车端恢复版本。共同主线 `main` 用于验收后的集成。新工作先读 [协作规则](AGENTS.md)，核对当前 Git 状态，再从最新 `origin/main` 开短期任务分支。原恢复分支 `fix/rtk-desktop-standard-grasp` 和迁移分支保留历史，恢复任务结束。

- **固定基线**由上述 annotated tag 标识，包含现场配置和交接。**车端部署**留在 `/home/ugv/slicer-ugv` 的恢复检出，核对时源码为 `82c03b4` 并带现场倒车配置。安装为增量更新的 `/home/ugv/slicer_noetic_c30d_v17/install`，`/home/ugv/slicer_noetic_current` 指向其工作区。后续 `main` 可通过 PR 向前演进。开发另用 checkout、构建和生成目录。入口见[运行手册](docs/OPERATIONS.md)和[构建说明](README.md)。
- **配套硬件**在基线及 `main` 的普通目录 `Hardware/`，随整车克隆取得，现场记录为 C30D v1.7。旧 `migration/native-noetic` 使用[独立硬件仓库](https://github.com/20DCYy06/slicer_ugv_hardware)的子模块。复现旧分支时在那个检出运行 `git submodule update --init -- Hardware`，其旧固定版本对应旧板和 v1.4。
- **能用的入口**包括 RTK 桌面与 odom/map 导航、无 RTK Gmapping/B 样条/TEB、正式视觉抓取、底盘与执行机构工具、V1/V2 空地接口。已有记录涵盖隔离导航与部分现场动作。最新部署的实车峰值、RTK map 和无 RTK 现场回归、桌面完整抓取接续及飞机端联合验收见[本次基线交接](docs/handoffs/NATIVE_RESTORED_BASELINE_20261122.md)。

1. 新功能从 `main` 开任务分支，随任务维护一份交接。
2. 现场有需求时安排剩余模式的针对性验证，记录实际软件和固件版本。
