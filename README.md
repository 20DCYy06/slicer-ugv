# Slicer UGV

中航工业杯整车项目。`main` 是 Ubuntu 20.04 / ROS1 Noetic 恢复后的共同主线。基线与实际部署见 [HANDOFF.md](HANDOFF.md)。

| 路径 | 职责 |
|---|---|
| `src/<包名>/` | 统一的 Catkin 源空间。包含传感器、定位、两种导航、控制、通信、空地任务与正式视觉伺服。 |
| `src/ugv_bringup/config/`、`Hardware/config.yaml` | 导航配置及底盘设备、几何输入。NoRTK 生成输入在 `src/ugv_bringup/config/no_rtk/`。 |
| `src/slicer_visual_grasp/` | 正式 Visual_Grasp 程序、标定、配套控制与算法测试。 |
| `tools/manual/` | 键盘、手柄和执行机构操作。 |
| `Hardware/` | 固定提交的 [slicer_ugv_hardware](https://github.com/20DCYy06/slicer_ugv_hardware) 子模块，维护底盘协议、主机驱动、解锁与固件。 |
| `deploy/udev/`、`docs/` | 设备规则、运行说明和交接。历史部署材料单独标识。 |

## 从克隆到构建

在 Ubuntu 20.04 / Noetic 环境，先按[依赖清单](docs/DEPENDENCIES.md)配置 APT、Livox SDK2 和视觉依赖，然后运行：

```bash
git clone --branch main https://github.com/20DCYy06/slicer-ugv.git
cd slicer-ugv
git switch -c feat/your-task
source /opt/ros/noetic/setup.bash
rosdep update --include-eol-distros
rosdep install --from-paths src --ignore-src --rosdistro noetic -y
export SLICER_NATIVE_WS="$HOME/slicer_noetic_your_task"
bash tools/build_native.sh "$SLICER_NATIVE_WS" -j2
bash tools/check_native.sh "$SLICER_NATIVE_WS"
source "$SLICER_NATIVE_WS/install/setup.bash"
```

基线的 `Hardware/` 随整车提交。历史迁移分支的硬件子模块获取方式见 [HANDOFF.md](HANDOFF.md)。

脚本直接从仓库 `src/` 构建，并把 NoRTK 运行配置、RTK 生成配置、构建与安装输出写入指定的外部工作区。检查包布局和配置生成时执行 `bash tools/check_native.sh --static`；只检查 Catkin 包布局时执行 `bash tools/check_native.sh --layout`。从旧布局升级时选用一个新的外部工作区。车端稳定检出和开发检出各有构建目录。

## 功能入口

- **无 RTK**用 Gmapping、B 样条和 TEB，Action 为 `/move_base`。后台启动和停止见[操作手册](docs/OPERATIONS.md#无-rtk-导航)。
- **RTK**用 DWA，Action 为 `/ugv/move_base`。先用 odom 导航配准，随后切到 map 导航。[RTK 现场操作](docs/rtk/field/AIR_GROUND_FIELD_RUNBOOK.md)
- **空地任务**以 V1 已验证接口为整合基线，V2 独立选用。V2 直接 launch 的 `monitor_only` 默认是 `false`，历史 Compose 默认是 `true`，原现场环境曾覆盖为 `false`。操作时显式写明。一套导航实例只启动一个任务执行器。[V1 接口](docs/rtk/air_ground_interface_v1.md) · [V2 协议](src/ag_uav_client/docs/protocol_v2.md)
- **视觉、底盘遥控和执行机构**见[操作手册](docs/OPERATIONS.md#独立硬件工具)，包含旧入口对照。

协作前看 [AGENTS.md](AGENTS.md)、[HANDOFF.md](HANDOFF.md) 和[开发接手建议](docs/MAINTENANCE.md)。
