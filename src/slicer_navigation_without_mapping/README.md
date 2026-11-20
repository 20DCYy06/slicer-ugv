# 雷达版在线导航入口

在仓库根目录使用 `tools/no_rtk.sh` 管理 NoRTK 会话。
`SLICER_NATIVE_WS` 选择构建工作区，默认为 `$HOME/slicer_noetic_dev`。

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
在线巡航读取本包的 `config/waypoints.yaml`，每次从空白 SLAM 启动都会重新建立地图原点。
