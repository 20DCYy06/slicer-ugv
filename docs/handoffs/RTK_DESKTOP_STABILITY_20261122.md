# RTK 导航限速与右栏稳定性 · 2026-11-22

- 从 `72c06d4` 在 `fix/rtk-desktop-stability` 独立工作树开发。原检出的无 RTK 速度参数与 `.vscode/` 保留。当前车端仍使用 `~/slicer_noetic_c30d_v17/install`。
- RTK 桌面从仓库加载 `slicer_field_navigation.launch`。odom 和 map 使用 `dwa_field.yaml` 中的 0.8 m/s、0.785398 rad/s 上限，Guard 按所选安装的生成配置和 DWA 参数取限。当前 v1.7 安装的 Guard 配置已支持 0.8 m/s。新构建由 RTK 生成器设置该上限。导航重启时读取新参数。
- 右栏固定单行布局。长提示省略显示并可悬停查看全文，普通状态短时波动由显示层缓冲。目标和当前位置以 WGS84 纬度、经度显示，未具备配准或 TF 时给出等待提示。Waypoint 终端输出也使用 WGS84。
- Nav Goal 在状态更新间隙可等候下一条可信的 Action 状态，最长 0.65 秒。实际派发仍要求 0.5 秒内的 Action 状态并重新校验导航帧、Mux、底盘和任务。重连与面板启停操作会取消尚在等待的目标。正在前往的坐标关联发送出去的 Action 目标。
- 通过 36 项桌面单测、10 项 launch 解析和 6 项运行配置生成及迁移测试。离屏 Qt 检查 256 和 480 像素宽的右栏。Xvfb 与独立 ROS Master 验证延迟心跳、停车拦截、目标经纬度、失联拒绝和打点。独立启动两种导航，Guard 日志确认 `forward=0.800 reverse=0.000 angular=0.785`。未重建现有安装，未运行实车。
- 重开 `./tools/rtk_desktop_ui.sh` 并重新启动导航即可加载。日志和参数入口见[整车操作手册](../OPERATIONS.md#rtk-本地分屏桌面)。切回 `fix/rtk-desktop-controls` 恢复上一版。
