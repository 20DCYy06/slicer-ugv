# 室内导航雷达与本地桌面 · 2026-11-21

- 从 `ce647aa` 在独立工作树开发。原检出 `fix/c30d-udev-replacement` 有未跟踪的 `.vscode/`。车端 `/home/ugv/slicer_noetic_current` 指向 `slicer_noetic_c30d_v16`，本次复用该安装及其航点工具、规划器和驱动二进制。
- 现场 run ID `b7233308-e5a7-11f1-818b-60ff9e774db9`。Livox SDK 报 `bind failed`。`eth0` 载波正常，但 DHCP 未配置 IPv4，MID360 的生成配置要求主机 `192.168.1.5`。启动前现在会检查配置中的主机绑定地址。待启用的 NetworkManager 配置为 `slicer-mid360` 配置，静态地址为 `192.168.1.5/24`，雷达为 `192.168.1.192`，继续用 Wi-Fi 默认路由。恢复原有有线配置时先停止使用雷达，再删除 `slicer-mid360` 并激活 `Wired connection 1`。
- `tools/no_rtk.sh nav --background` 使用同用户 Jetson 本地 X11 会话；可显式传 `gui:=false`。`tools/no_rtk.sh rviz` 为运行中的本机 ROS 导航单独启动界面。RViz 的 `2D Nav Goal` 发布 `/move_base_simple/goal`；`Record Waypoint` 发布 `/clicked_point`，录点节点保存到在线巡航读取的 `src/slicer_navigation_without_mapping/config/waypoints.yaml`。

## 验证

- 静态：Shell 语法、两类 launch 节点、MID360 缺地址时的报错和原生参数预检通过。launch、RViz 配置与包装脚本从源码读取，未重编译现有安装。
- 隔离 ROS：无底盘设备，网络仅 loopback。模拟扫描及底盘通过 SLAM、B 样条/TEB、Guard 和 Mux 验证。RViz 使用本地 X11、Mesa 软件渲染和独立 ROS master，实际连续鼠标点击 10 次均写入测试 YAML。
- 车端：从带转发 `DISPLAY` 的环境验证实际 RViz 窗口出现在本地 `:0`。核对目标与录点 ROS 连接，未发送现场运动目标。雷达网口配置待本机管理员认证，点云频率和现场地图待确认。

操作见 [整车操作手册](../OPERATIONS.md)。源码回退时撤销本次修复提交，原 Catkin 安装和网络配置可独立选择。
