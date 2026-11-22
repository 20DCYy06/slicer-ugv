# 雷达版在线导航入口

本包组合无 RTK 在线 Gmapping、B 样条和 TEB。开发时在所选工作区的整车源码根目录运行
`tools/no_rtk.sh`。默认使用该检出对应的工作区。正式运行通过
`~/workspace/slicer/run no-rtk` 选择 `current` 对应的发布。
图形窗口使用同一用户已登录的 Jetson 本地 X11 桌面。导航已运行时可用
`tools/no_rtk.sh rviz` 打开开发工作区桌面工具。

```bash
bash tools/no_rtk.sh build
bash tools/no_rtk.sh check
bash tools/no_rtk.sh nav --background gui:=false start_control:=false
bash tools/no_rtk.sh stop
```

现场操作员按实车流程启动底盘和发起目标或巡航。指定 `start_control:=false` 可查看
雷达和在线地图。脚本加载选定工作区的 Catkin overlay，沿用当前 ROS 网络环境。
后台日志位于开发工作区的 `logs/no_rtk/`，或正式发布的 `logs/<release-id>/no_rtk/`。
`stop` 向同一工作区或发布登记的
roslaunch 进程发送 SIGINT。操作目标、巡航和地图的入口见 `bash tools/no_rtk.sh help`。

参数源文件位于 `src/ugv_bringup/config/no_rtk/`，构建时生成的运行参数位于
工作区的 `runtime/no_rtk/`。车端串口配置来自生成的 `runtime/no_rtk/config/c30d_host.yaml`，由所选硬件仓默认值或显式现场底盘 YAML 生成。
RViz 的 **2D Nav Goal** 发送导航目标，**Record Waypoint** 连续录点到运行状态目录的
`waypoints.yaml`。开发默认用工作区的 `runtime/no_rtk/`，正式发布用
`site/state/<release-id>/no_rtk/`。录点和在线巡航共用该文件。仅在无界面且未运行录点节点时单独使用
`tools/no_rtk.sh record`。每次从空白 SLAM 启动都会重新建立地图原点。
