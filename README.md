# Slicer UGV

中航工业杯整车项目。共同主线使用 Ubuntu 20.04 / ROS1 Noetic 和 Catkin。当前主线、发布候选、车端实际运行和历史基线见 [交接](HANDOFF.md)。

| 路径 | 职责 |
|---|---|
| [`src/` 包索引](src/README.md) | 整车自有或集成的 ROS 包。车端定位、导航、控制仲裁、通信、空地任务和正式视觉伺服。 |
| `src/ugv_bringup/config/` | 整车参数。RTK 与无 RTK 使用各自的配置。 |
| `tools/manual/` | 组合遥控、手柄和执行机构入口。 |
| `dependencies.repos` | 硬件、FAST-LIO、Livox 驱动和 SDK2 的路径、公开来源与完整 SHA。 |
| `deploy/udev/`、`docs/` | 设备规则、运行手册和任务交接。 |

## 从克隆到构建

先按[依赖说明](docs/DEPENDENCIES.md)准备 Focal/Noetic、rosdep 和 `python3-vcstool`。每个工作区各自获取固定依赖，从 `W/src` 组合构建。

```bash
BASE="$HOME/workspace/slicer"
W="$BASE/main"
mkdir -p "$W/src" "$BASE/data"
git clone --branch main https://github.com/20DCYy06/slicer-ugv.git "$W/src/slicer-ugv"
cd "$W"
vcs import . < src/slicer-ugv/dependencies.repos

source /opt/ros/noetic/setup.bash
rosdep update --include-eol-distros --rosdistro noetic
rosdep install --from-paths "$W/src" --ignore-src --rosdistro noetic -y
cd "$W/src/slicer-ugv"
bash tools/check_native.sh --static "$W"
bash tools/check_offline.sh
bash tools/build_native.sh "$W" -j2
bash tools/check_native.sh "$W"
source "$W/install/setup.bash"
```

整车检出在 `W/src/slicer-ugv`，ROS 第三方检出在 `W/src/third_party`。独立硬件仓位于 `W/deps/slicer_ugv_hardware`，SDK2 源码位于 `W/deps/Livox-SDK2`。SDK 在 `W/sdk` 本地编译安装，Catkin 产物、生成配置和日志分别写入 `W/{build,devel,install}`、`W/runtime` 和 `W/logs`。

新任务从已更新的本地主线开短期分支，使用 `dev/<任务名>` 和独立的依赖检出。

```bash
D="$BASE/dev/your-task"
mkdir -p "$D/src"
git -C "$W/src/slicer-ugv" fetch origin main
git -C "$W/src/slicer-ugv" worktree add -b feat/your-task "$D/src/slicer-ugv" origin/main
(cd "$D" && vcs import . < src/slicer-ugv/dependencies.repos)
cd "$D/src/slicer-ugv"
bash tools/build_native.sh "$D" -j2
```

恢复基线的 `Hardware/` 随整车 Git 树。更早的 `migration/native-noetic` 使用硬件子模块。复现历史版本时按相应提交的说明操作。

## 功能入口

- **无 RTK**使用 Gmapping、B 样条和 TEB，Action 为 `/move_base`。后台启停及打点见[操作手册](docs/OPERATIONS.md#无-rtk-导航)。
- **RTK 与自动配准**使用 DWA 和 `/ugv/move_base`。共享导航在首个 WGS84 任务需要时自动配准并接续原任务。可用桌面、CLI 或选一个空地执行器。[使用与诊断](docs/OPERATIONS.md#rtk-导航与空地接口)
- **空地任务**保留已验证的 V1 和整车集成的 V2 JSON。V2 启动时显式指定 `monitor_only`。飞机团队独立 `AG_Client` 的较新 typed V2 有自己的工作区和消息版本。[V1 接口](docs/rtk/air_ground_interface_v1.md) · [V2 协议](src/ag_uav_client/docs/protocol_v2.md)
- **正式启动、视觉和遥控**见[操作手册](docs/OPERATIONS.md)。正式入口使用 `~/workspace/slicer/current` 选定的发布及其对应安装。

协作前看 [AGENTS.md](AGENTS.md)、[HANDOFF.md](HANDOFF.md) 和[维护说明](docs/MAINTENANCE.md)。
