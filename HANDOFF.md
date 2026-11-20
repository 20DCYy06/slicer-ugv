# Slicer UGV 原生迁移交接

当前目标分支是 `migration/native-noetic`。本轮从 `c80b68b436ec18ff4a31bb625f99a007239cccdf` 在 `refactor/ros-layout` 继续。源码统一到 `src/<包名>`，控制、TF、消息和两个导航 Action 延续各自基线。安装工作区与源码检出分开。

开始使用请读 [构建与克隆](README.md) 和 [操作手册](docs/OPERATIONS.md)。本轮迁移映射、恢复来源和验证见[任务交接](docs/handoffs/ROS_LAYOUT_20261120.md)。旧盘与原始工作区的 bundles、补丁、未跟踪源码和 v1.4 恢复镜像延续上一轮本地恢复目录。

1. 在 Ubuntu 20.04 ARM64 目标机用独立工作区安装，核对雷达、GNSS、底盘、手柄与相机的实际设备和权限。
2. 使用明确的版本和模式分别验证无 RTK/RTK、后台启停和 FKIE 双机通信。
3. 现场验证组合手柄、正式视觉伺服和实车导航，再确立稳定 `main`。
