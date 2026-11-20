# Slicer UGV 原生迁移交接 · 2026-11-20

仓库为 `/home/rescue/slicer_ugv_migration`，分支 `migration/native-noetic`。延续 `a5df2ad` 提交链。阶段提交按顺序为 `65f3de7` 保留现场改动、`4453e3d` 引入快照选定模块、`d579f7e` 恢复旧盘固件输入及部署配方、`fbdbe23` 适配原生生成、安装和启动。构建中补齐 V2 测试入口和 FAST-LIO 头文件，并允许源码安装时省略可选的保存地图。

V1 按已验证部署 `4a53b66` 作为原生执行基线。V2 来自旧盘 AG_Client `a2003d4`，镜像安装副本与选查源码一致。旧盘 11 月 19 日日志记录先启动 monitor-only，随后以 `monitor_only=false` 重启并建立话题连接，最后停止该会话。该日志只覆盖进程和通信。V2 的并行软件验证使用仅观测模式，启动方式见 [README](README.md)。

Git bundles、六棵旧根工作树状态与补丁、未跟踪源码和 v1.4 镜像及整片读回，保存在 `/home/rescue/ugv-migration-20261120/local-recovery`。来源与版本冲突见 `/home/rescue/ugv-migration-20261120/MIGRATION_INVENTORY.md`。保存材料留在本地，便于核对和回退。

**验证。** WSL Jammy 源码预检通过。隔离 Ubuntu 20.04.5 amd64 / Noetic 已通过 14 包 Catkin 构建与安装，检查了 FAST-LIO 和 Livox 可执行文件、V1 消息 MD5，以及定位、两种导航、底盘、V1 和 V2 的 launch 解析。空地客户端与路由定向通过 56 个用例（34 个单元、22 个使用模拟 ROS 接收端）。v1.4 主机 C 与协议测试通过。裸机目标、ARM64、真实 FKIE 联机和设备及导航实测待对应环境验证。

**GitHub。** 已查看连接账号 `20DCYy06`，目标 `slicer-ugv` 在该连接下返回 404。本机没有登录的 gh，SSH 使用 GitHub 官方主机密钥验证后以 publickey 拒绝。未创建仓库或推送。

1. 在 Focal/Noetic 裸机按 [依赖清单](docs/DEPENDENCIES.md) 安装并复验 `build_native.sh` 与 `check_native.sh`，为稳定运行另建独立检出及工作区。
2. 恢复车端网络、设备规则、正式 GNSS 服务及现场外参，分别验证两种导航，并选定 V1 或 V2 执行入口。
3. 本机登录本人 GitHub 后，确认目标私有仓库归属及可用性，推送本迁移分支。
