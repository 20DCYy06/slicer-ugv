# 整车原生恢复基线 · 2026-11-22

## 范围与版本

队友现场确认当前车端版本基本稳定，团队决定用它结束恢复阶段。固定基线为 `baseline/native-restored-20261122`。`main` 承担之后的稳定集成。原恢复分支 `fix/rtk-desktop-standard-grasp` 和旧迁移分支保留历史。恢复任务完成，独立需求另开分支。

- **车端源码与部署**。实际检出 `/home/ugv/slicer-ugv`，起点 `82c03b4bb8a8803d56afb5d2f9655ee979cf8761`。`/home/ugv/slicer_noetic_current` 指向 `/home/ugv/slicer_noetic_c30d_v17`，操作入口 source 其 `install/setup.bash`。核对了最近部署的视觉模块、控制节点、规划器和运行配置。它是 v1.7 工作区上的增量安装，整车安装目录没有统一构建 SHA。本次收尾未重建、未换现场检出、未更新实车安装。
- **稳定配置**。补交了实际无 RTK 前进 0.8、倒车 0.5 m/s 中唯一未提交的源码参数 `max_reverse: 0.5`，与 `runtime/no_rtk` 实际值一致。RTK DWA 的有效倒车上限仍为 0。现场抓取阈值 30 px 在 `~/.config/slicer_visual_grasp/config.json`，历史 20 px 记录对应此前配置。相机标定和其他现场参数保留在外部配置。GNSS 外部入口为 `~/.config/slicer/gnss.yaml`，凭据由其受保护文件路径引用。
- **硬件**。基线的 `Hardware/` 是普通跟踪目录，Git 树 `a4d88ebcec71a81904d82719c30ffd01648788cd`，随整车提交取得。最近改动该目录的整车提交为 `82c03b4`。C30D v1.7 对应板号 `5CF0016377`，归档镜像 SHA-256 `4e9bcafeab040d7f1bd0ad8758948bc180e1c231afa247f8c492fd8a21e4431a`。记录中的烧录后读回 SHA-256 `81067fa115fdcc426f6b4d3e2ef50517ef6767a4f442877db54052e45c4d7153`。固件来源和断动力安装记录见 [v1.7 发布说明](../../Hardware/Firmware/v1.7/README.md)与 [安装记录](../../Hardware/last_install.json)。
- **远端整合**。整车旧默认 `migration/native-noetic` 为 `5e0007a8982b45dd60f9cece3c0d3a69cbf18572`，与现场起点分叉 3/46 个提交。基线通过普通合并保留双方历史，保留远端的布局检查、维护说明及 rosdep 镜像提示。旧迁移分支的 [独立 Hardware 子模块](https://github.com/20DCYy06/slicer_ugv_hardware/commit/9ce2c29d5c3bd27392ea8c7a652348816b6a26cf)仍固定 `9ce2c29d5c3bd27392ea8c7a652348816b6a26cf`，只有旧板配套固件，不能替代现场 v1.7。复现旧分支时在其检出运行 `git submodule update --init -- Hardware`。此基线无需另取硬件提交。
- **现场未提交内容**。车端原检出仍保留其配置工作副本和 `.vscode/settings.json`。原配置的倒车值已经进入共同基线。IDE 本机设置、`Hardware/Backups/`、`Hardware/Logs/`、缓存和受保护的用户配置留在车端。`main` 的后续提交按任务交接记录，车端实际部署版本按每次部署另记。

## 验证依据与边界

| 类型 | 结果与证据 |
| --- | --- |
| 队友现场确认 | 团队确认采用这版作为恢复基线。确认范围为版本基本稳定，具体测试范围以下列记录为准。 |
| 本次静态核对 | 核对两侧 Git 历史、独立检出、ROS 包布局和双模式生成配置。v1.7 源文件清单 30 项及归档固件 SHA-256 匹配。 |
| 已有构建和隔离测试 | `82c03b4` 对应的隔离 Catkin 安装、53 项桌面、55 项视觉通过（1 项缺录制帧跳过）、15 项 RTK Guard、16 项无 RTK 驱动，并有 Xvfb/ROS、Gmapping、地图重置和三模式 7 m 模拟直行记录。隔离速度达 0.800 m/s，RTK 障碍和超时停车通过。[测试与部署记录](RTK_STANDARD_GRASP_NAV_SPEED_20261122.md) |
| 已有真实设备和实车记录 | 旧记录覆盖 v1.7 的烧录、读回和零输出。2026-11-22 05:22–05:28 的新部署日志记录 RTK odom 实车到点、C30D 就绪、桌面恢复释放第二次获 ACK，及独立视觉窗口按 30 px 完成抓取、抬升和释放。证据为车端 `~/.ros/log/rtk-desktop-odom-knz_b4n_.log:214`、`rtk-desktop-driver-uvmjcvj5.log:1`、`rtk-desktop-vision-k2h6ilj2.log:3–11` 和 `~/.config/slicer_visual_grasp/data/runtime.log:1106,1176–1184`。 |
| 尚未验证 | 当前部署的 RTK map、无 RTK 实车导航、真实峰值和 PWM、桌面抓取后持物继续导航并释放的完整现场链路，及 V1/V2 与飞机的最新联合验收。桌面日志有第一次 AirGrip 关泵未应答的记录。0.8 m/s 与 PWM 89.31% 是配置及隔离证据，实际推力和峰值尚待测。本次没有新增实车运动验收。 |

## 启动、恢复与下一步

- RTK 桌面 `./tools/rtk_desktop_ui.sh`，无 RTK `./tools/no_rtk.sh nav`，正式独立视觉 `rosrun slicer_visual_grasp run.sh --run`。启动前核对 `SLICER_NATIVE_WS` 和已 source 的 overlay。整车配置在 `src/ugv_bringup/config/`，具体操作及诊断见[操作手册](../OPERATIONS.md)。
- 源码回到固定基线标签可复现整车/硬件组合。车端增量安装回退使用 `/home/ugv/slicer_noetic_c30d_v17/nav-visual-backups/standard-grasp-20261122.vytbqo9e`，按相对路径恢复文件，移除其 `new-files` 所列新增文件。使用现有[烧录和验收记录](../../Hardware/Firmware/v1.7/README.md)确认固件版本，再决定是否需要硬件回退。
- 接手的第一个动作是读根交接并核对 `git status --short --branch`、`git rev-parse HEAD`、`git ls-tree HEAD Hardware` 和车端 overlay。接下来从最新 `origin/main` 创建独立任务分支。确有现场需求时，补测上述剩余链路并把结果写进相应任务交接。
- GitHub 交付核对以远端 refs 和 Settings 为准。`main` 目标规则为 PR 合入、禁强推和删除、0 人强制审批、空 bypass，仅在当前工作流成功后要求 `prepare`。本文件记录基线内容，GitHub 设置是可独立变化的仓库状态。

### 新窗口接手提示词

> 在 `/home/ugv/slicer-ugv` 先读 `AGENTS.md`、`HANDOFF.md` 和本基线交接。核对当前 Git 分支、远端、未提交改动和实际 ROS overlay。新功能从最新 `origin/main` 用独立工作树开短期分支，使用独立 Catkin 目录。沿用已有现场证据，分开记录静态、构建、模拟和实车结果，并更新本任务的交接。
