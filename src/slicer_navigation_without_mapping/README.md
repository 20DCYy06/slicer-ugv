# 雷达版在线导航入口

在仓库根目录使用 `tools/no_rtk.sh` 管理 NoRTK 会话。
`SLICER_NATIVE_WS` 显式选择工作区。运行时默认使用存在的 `$HOME/slicer_noetic_current`，其余情况下使用 `$HOME/slicer_noetic_dev`。构建默认使用开发工作区。图形窗口使用同一用户已登录的 Jetson 本地 X11 桌面。导航已运行时用 `tools/no_rtk.sh rviz` 打开本地桌面工具。

```bash
bash tools/no_rtk.sh build
bash tools/no_rtk.sh check
bash tools/no_rtk.sh nav --background gui:=false start_control:=false
bash tools/no_rtk.sh stop
```

现场操作员按实车流程启动底盘和发起目标或巡航。指定 `start_control:=false` 可查看
雷达和在线地图。脚本加载选定工作区的 Catkin overlay，沿用当前 ROS 网络环境。
后台输出写在该工作区的 `runtime/no_rtk/roslaunch.log`，`stop` 向同一工作区登记的
roslaunch 进程发送 SIGINT。操作目标、巡航和地图的入口见 `bash tools/no_rtk.sh help`。

参数源文件位于 `src/ugv_bringup/config/no_rtk/`，构建时生成的运行参数位于
工作区的 `runtime/no_rtk/`。车端串口配置来自 `Hardware/config.yaml`。
RViz 的 **2D Nav Goal** 发送导航目标，**Record Waypoint** 连续录点到本包的
`config/waypoints.yaml`。录点和在线巡航共用该文件。仅在无界面且未运行录点节点时单独使用
`tools/no_rtk.sh record`。每次从空白 SLAM 启动都会重新建立地图原点。
