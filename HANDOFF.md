# Slicer UGV 原生迁移交接 · 2026-11-20

仓库为 `/home/rescue/slicer_ugv_migration`，分支 `migration/native-noetic`。延续 `a5df2ad` 提交链。阶段提交按顺序为 `65f3de7` 保留现场改动、`4453e3d` 引入快照选定模块、`d579f7e` 恢复旧盘固件输入及部署配方、`fbdbe23` 适配原生生成、安装和启动。

V1 车端保留独立部署对照 `4a53b66`。V2 来自旧盘 AG_Client `a2003d4`，镜像安装副本与选查源码一致。旧盘 11 月 19 日日志记录先启动 monitor-only，随后以 `monitor_only=false` 重启并建立话题连接，最后停止该会话。该日志只覆盖进程和通信。默认执行器与通信模式按 [README](README.md) 显式选用。

Git bundles、六棵旧根工作树状态与补丁、未跟踪源码和 v1.4 镜像及整片读回，保存在 `/home/rescue/ugv-migration-20261120/local-recovery`。来源和版本冲突的完整记录仍在该目录的 `MIGRATION_INVENTORY.md`。保存材料留在本地，便于核对和回退。

**验证。** WSL Jammy 的源码预检通过，两个配置生成器运行成功，Catkin 包发现和拓扑排序为 14 个包，其中特定消息包只有一份。v1.4 的主机 C 与协议测试通过。完整 Noetic 安装与 launch 结果见迁移盘点。裸机目标、ARM64 和设备实测未执行。

**GitHub。** 已查看连接账号 `20DCYy06`，目标 `slicer-ugv` 在该连接下返回 404。本机没有登录的 gh，SSH 使用 GitHub 官方主机密钥验证后以 publickey 拒绝。未创建仓库或推送。

1. 在 Focal/Noetic 裸机按 [依赖清单](docs/DEPENDENCIES.md) 安装并复验 `build_native.sh` 与 `check_native.sh`，为稳定运行另建独立检出及工作区。
2. 恢复车端网络、设备规则、正式 GNSS 服务及现场外参，分别验证两种导航，并选定 V1 或 V2 执行入口。
3. 本机登录本人 GitHub 后，确认目标私有仓库归属及可用性，推送本迁移分支。
