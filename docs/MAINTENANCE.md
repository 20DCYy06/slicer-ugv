# 开发接手建议

## 接手与交付

- 开工先核对整车分支、已有改动、`dependencies.repos`，以及现场实际运行的整车和固件版本。ROS 终端再核对所 source 的 Catkin overlay。开发检出、构建目录和车端稳定检出各自独立。
- 改动放短分支，PR 写清症状、改动、实际验证、回退方式和待办。独立仓库的依赖变更先发布子仓，再在整车 PR 更新完整 SHA。复现历史硬件子模块时按对应旧分支说明操作。
- 每项任务及时更新自己的交接文件。合入或共同状态变化时更新 [HANDOFF.md](../HANDOFF.md)。现场证据记录日期、板号、固件 SHA256、生效配置和输出位置，材料留在约定的发布或本地归档处。实车日志、备份和凭据使用受保护的位置。

## 保住接口边界

- 局部 EKF 负责 `odom → base_link`。无 RTK 的 Gmapping 和 RTK 的全局 EKF 分别在各自模式发布 `map → odom`。无 RTK Action 为 `/move_base`，RTK Action 为 `/ugv/move_base`。
- ROS 导航输出走 Guard/Mux，Mux 独占 `/cmd_vel`。正式视觉和独立遥控工具沿用已记录的直接设备接口。切换时保持停车，确认仅有一个底盘串口所有者。改协议、设备映射或几何前核对实际板号和版本。
- 同一导航实例只运行一个空地任务执行器。V1 是已验证的整合基线。使用 V2 时显式设置 `monitor_only`，旧 Compose 和原生 launch 的默认值有差异。双机联调核对时钟、节点可回连地址和消息 MD5，再做飞机到车端的消息往返。轨迹、停车和地理到点各自记录。[空地接口](rtk/air_ground_interface_v1.md)

## 验证到什么程度就写什么

- 分开记录静态解析、隔离构建、仿真或回放、目标机 ARM64、真实设备和实车运动。旧 V1 的 ARM64 安装与局域网回执只对应当时提交。当前版本的完整证据随本次构建和实测追加。
- 桌面开发可以运行 `bash tools/check_native.sh --static` 确认两种配置生成。CI 的 `prepare` 检查包布局及两套生成配置。
- 目标机先按 [依赖清单](DEPENDENCIES.md) 初始化 rosdep、获取固定源码依赖，再构建工作区 SDK2、ROS 包和验设备。Focal 与 Jammy 的库分别构建。现场按[操作手册](OPERATIONS.md)核对模式、设备权限、定位状态和停车方式。


## 候选发布与切换

在最终发布路径干净构建。候选的 `release.json` 记录整车和依赖完整 SHA、构建环境、配置摘要、已有固件证据及验证状态。系统包版本单独随发布记录保存。

```bash
BASE="$HOME/workspace/slicer"
W="$BASE/releases/<release-id>"
mkdir -p "$W/src"
git clone https://github.com/20DCYy06/slicer-ugv.git "$W/src/slicer-ugv"
git -C "$W/src/slicer-ugv" checkout --detach <vehicle-sha>
(cd "$W" && vcs import . < src/slicer-ugv/dependencies.repos)
cd "$W/src/slicer-ugv"
bash tools/check_native.sh --static "$W"
bash tools/check_offline.sh
bash tools/build_native.sh "$W" -j2
bash tools/check_native.sh "$W"
```

在设备隔离的环境中运行适用的 Noetic ROS 验证。首 WGS84 CLI 接续用已 source 的发布安装运行 `python3 src/ugv_navigation/test/check_auto_registration_ros.py cli`，测试进程自建 loopback master。正式启动和新现场验收由现场负责人协调。

首次部署选择器，使用该发布对应源码的 `tools/run_release.sh` 安装为 `$BASE/run`。切换前先核对候选验收记录，停止旧入口，验证旧导航和底盘退出，保存原链接。使用同一文件系统的临时链接替换 `current`，然后确认解析的安装目录、LiDAR、GNSS/odom、控制链和任务状态。

```bash
install -m 0755 "$W/src/slicer-ugv/tools/run_release.sh" "$BASE/run"
readlink "$BASE/current"  # 记录已存在的原发布链接
ln -s "releases/<release-id>" "$BASE/.current-next"
mv -T "$BASE/.current-next" "$BASE/current"
"$BASE/run" status
"$BASE/run" shell rospack find ugv_navigation
```

通过后用 `run rtk` 或 `run no-rtk nav --background` 启动。异常先用同一选定版本停止新入口，按上述方法切回前一个链接，再检查后启动。首次切换回旧增量部署用旧源码入口和原现场 overlay。目录回退只选择旧软件，固件版本、标定和外部现场配置按各自的独立记录恢复。
