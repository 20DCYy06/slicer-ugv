# 无 RTK 启动修复 · 2026-11-20

从 `fix/c30d-udev-replacement` 的 `4b37367` 开始，原工作区有未跟踪的 `.vscode/`。独立修复分支为 `fix/no-rtk-startup`。现场日志的 run ID 为 `4ba0d118-e501-11f1-bebb-60ff9e774db9`。启动回退到了 `/home/ugv/slicer_noetic_dev/devel`，缺少 Livox、FAST-LIO、点云臂过滤和速度 Guard 的可执行文件。无显示的终端还启动了 RViz 和目标坐标窗口。

在同一日志中，底盘完成握手后 66 ms 即报轮速遥测超时。隔离用例复现了先收到 0x93、第一帧 0x90 尚未收到时，原握手会提前返回。共享 Host 握手现在等待本次连接收到的有效 IMU 与新鲜轮速状态，运行期超时停车条件仍沿用原值。现场只收串口采样持续 4 秒，0x90 和 0x93 各收到 100 帧，CRC 错误 0。

车端 `/home/ugv/slicer_noetic_current` 指向完整的 `/home/ugv/slicer_noetic_c30d_v16`。从仓库根运行 `./tools/no_rtk.sh nav` 会优先用该安装，无 `DISPLAY` 时自动关闭两个图形窗口。可用 `SLICER_NATIVE_WS` 指定另一套完整安装。启动前按所传参数解析 launch，并检查参与启动的节点可执行。默认构建仍使用开发目录。复用 v1.6 安装，NoRTK 驱动在运行时读取当前源码中的共享 Host。

离线回归通过，无 RTK 驱动 13 项、融合手柄 11 项、独立工具 14 项。启动检查复现并拦截旧工作区的四项缺失，新安装可以解析无界面和显式 GUI 两类参数。无设备、仅回环网络的独立 ROS master 上，通过了无界面模拟启动、move_base、Guard 和模拟底盘就绪检查。现场雷达输入和带动力导航待实测。

需要回退时先停当前入口，使用 `SLICER_NATIVE_WS` 指定另一套与当前底盘固件匹配的完整安装。源码和 README 的改动可按本次提交回退，车端默认安装链接可由现场人员单独撤销。
