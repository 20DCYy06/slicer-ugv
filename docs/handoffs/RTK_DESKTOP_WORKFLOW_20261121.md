# RTK 本地桌面流程控制与差分配置

- 从 `c62a6ad` 在 `feat/rtk-desktop-workflow` 的独立工作区开发。原检出中的无 RTK 参数修改与 `.vscode/` 保留。分屏直接使用所选 v1.7 ROS 安装，Python 和 shell 入口从仓库运行。业务包及原启动文件未改动。
- 现场新增 `~/.config/slicer/gnss.yaml` 和权限 `0600` 的 `~/.config/slicer/ntrip_credentials.json`。YAML 使用核实过的 USB 稳定路径和原 115200 波特率。NTRIP 是 `203.107.45.154:8002/AUTO`，按现场指定使用 WGS84。凭据留在车端。现场使用 `cup_gnss` 读取器及 `roslaunch --dump-params` 核实配置结构，没有连接 caster。
- 面板通过标准 Master、ROS 状态新鲜度及本机进程发现外部启动。外部启动的步骤在原终端关闭。面板启动的子进程独立成组，关闭面板或全部停止时按流程退出。配准使用现有 FIXED 门槛和直行确认，切换到 `map` 时保留定位与底盘。视觉窗口放在左侧，检测到可见窗口及 V4L2 打开后变绿。抓取动作需要底盘串口空闲。
- 静态检查和 28 项离线测试通过。`tools/rtk/desktop/tests/check_workflow_desktop.sh` 在 Xvfb 和独立 loopback ROS master 验证 3:1 分屏、无 master 启动与退出、模拟相机窗口定位、外部窗口保留，以及原有 RViz `odom`/`map` 目标与打点。未执行实车配准、导航、抓取或飞机联调。
- 操作和环境变量见 [整车操作手册](../OPERATIONS.md#rtk-本地分屏桌面)。可用 Git 撤销本次提交以恢复前一版桌面。现场 NTRIP 配置在 Git 之外，可在切换会话时另行归档或更换。面板与终端可以管理不同步骤，同一步骤启停需顺序执行，既有手动命令未加共享启动锁。
