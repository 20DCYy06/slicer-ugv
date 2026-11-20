# RTK 导航工作区

整车协作和现场授权遵循仓库根目录的 [AGENTS.md](../../AGENTS.md)。原生构建从仓库根目录使用 `tools/build_native.sh`，当前目录的 `run.sh` 是旧容器入口。

- ROS 接口沿用标准 Topic、Service 和 Action，整车参数放在 `ugv_bringup/config`。私有 GNSS 配置与凭据由现场在源码之外提供。
- 速度指令接入 Slicer Guard 的 raw 入口，Mux 负责最终 `/cmd_vel`。只启用一个任务执行器，并分别核对局部和全局 TF 的发布者。
- 旧项目和现场原始证据保持只读。离线模拟使用独立 ROS master、无设备环境，结果与实际 ROS 和实车诊断分别记录。
