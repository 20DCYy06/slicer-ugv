# 视觉抓取启动与退出修复 · 2026-11-21

- 从 `78f64e9` 在独立工作树和独立 Catkin 目录修复。原 Jetson 检出的无 RTK 速度编辑和 `.vscode/` 保留。当前安装指向 `/home/ugv/slicer_noetic_c30d_v16`。
- 现场旧 Bash 启动配置只加载系统 Noetic，`rospack` 无法找到视觉包。Jetson 的 `~/.bashrc` 现在会在新终端加载当前整车安装，原文件备份为 `~/.bashrc.before-slicer-visual-grasp`。已打开的旧终端可执行 `source "$HOME/slicer_noetic_current/install/setup.bash"`。
- 加载正确环境后复现真实预览退出异常。关闭窗口现在等待相机采集线程释放设备，再离开 Tk 主循环；耗时异常仍走原有有界退出逻辑。图像对象绑定到对应的 Tk 画布。动作模式复用手动工具已有的 `dialout` 组切换，预览仍在 Jetson 本地 X11 上运行。
- 静态和离线：独立视觉包构建安装及安装后的 `rosrun ... --check` 通过。模拟逻辑 40 项通过，1 项录制帧回放无样本而跳过；无相机及串口设备、隔离网络的 UI 回归 11 项通过。部署仅更新视觉包自身文件及共享支持副本。

启动和观测见 [视觉抓取说明](../../src/slicer_visual_grasp/README.md)。原安装备份与现场相机验证在部署后记录。
