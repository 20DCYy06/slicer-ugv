# 开发接手建议

## 接手与交付

- 开工先核对整车分支、已有改动、`git ls-tree HEAD Hardware`，以及现场实际运行的整车和固件版本。ROS 终端再核对所 source 的 Catkin overlay。开发检出、构建目录和车端稳定检出各自独立。
- 改动放短分支，PR 写清症状、改动、实际验证、回退方式和待办。基线的 `Hardware/` 由整车版本固定。复现旧迁移分支时同时记录整车和硬件子模块 SHA，并初始化该分支固定的子模块。
- 每项任务及时更新自己的交接文件。合入或共同状态变化时更新 [HANDOFF.md](../HANDOFF.md)。现场证据记录日期、板号、固件 SHA256、生效配置和输出位置，材料留在约定的发布或本地归档处。实车日志、备份和凭据使用受保护的位置。

## 保住接口边界

- 局部 EKF 负责 `odom → base_link`。无 RTK 的 Gmapping 和 RTK 的全局 EKF 分别在各自模式发布 `map → odom`。无 RTK Action 为 `/move_base`，RTK Action 为 `/ugv/move_base`。
- ROS 导航输出走 Guard/Mux，Mux 独占 `/cmd_vel`。正式视觉和独立遥控工具沿用已记录的直接设备接口。切换时保持停车，确认仅有一个底盘串口所有者。改协议、设备映射或几何前核对实际板号和版本。
- 同一导航实例只运行一个空地任务执行器。V1 是已验证的整合基线。使用 V2 时显式设置 `monitor_only`，旧 Compose 和原生 launch 的默认值有差异。双机联调核对时钟、节点可回连地址和消息 MD5，再做飞机到车端的消息往返。轨迹、停车和地理到点各自记录。[空地接口](rtk/air_ground_interface_v1.md)

## 验证到什么程度就写什么

- 分开记录静态解析、隔离构建、仿真或回放、目标机 ARM64、真实设备和实车运动。旧 V1 的 ARM64 安装与局域网回执只对应当时提交。当前版本的完整证据随本次构建和实测追加。
- 桌面开发可以运行 `bash tools/check_native.sh --static` 确认两种配置生成。CI 的 `prepare` 检查包布局及两套生成配置。
- 目标机先按 [依赖清单](DEPENDENCIES.md) 初始化 rosdep、安装固定版本的 Livox SDK2，再构建和验设备。Focal 与 Jammy 的库分别构建。现场按[操作手册](OPERATIONS.md)核对模式、设备权限、定位状态和停车方式。
