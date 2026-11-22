# ROS 包导航

先按车辆运行模式选入口，再找对应的包。

开发行为树从 [BT 行为树开发接入指南](../docs/BT_START_HERE.md) 开始。

| 运行模式 | 日常入口 | Action |
| --- | --- | --- |
| 有 RTK，含首任务自动配准 | [RTK 桌面或共享导航 launch](../docs/OPERATIONS.md#rtk-导航与空地接口) | `/ugv/move_base` |
| 无 RTK，雷达在线建图 | [`tools/no_rtk.sh nav`](../docs/OPERATIONS.md#无-rtk-导航) | `/move_base` |

## 名字相近的包

| 包 | 实际职责 |
| --- | --- |
| [`ugv_navigation`](ugv_navigation/) | RTK 地理配准、WGS84 目标、共享导航运行器、V1 车端执行器，以及 RTK 的 `move_base` 接入。 |
| [`slicer_navigation`](slicer_navigation/) | 提供无 RTK 使用的 B 样条插件、TEB 参数、速度保护、巡航和 RViz 打点。车体点云和扫描滤波也供两种模式的传感器链使用。保留静态地图导航与建图入口。 |
| [`slicer_navigation_without_mapping`](slicer_navigation_without_mapping/) | 封装日常无 RTK 启动，组合传感器、在线 Gmapping、规划及桌面工具。包名来自旧入口，日常模式会实时建图。 |
| [`ugv_bringup`](ugv_bringup/) | 整车 launch 和参数。RTK 传感器、底盘和导航入口，以及 `config/no_rtk/` 的无 RTK 参数输入，都在这里。 |
| [`slicer_bringup`](slicer_bringup/) | 两种模式共用的雷达、FAST-LIO、局部 EKF 和点云到扫描链路。 |

## 其他入口

- 底盘控制与仲裁看 [`ugv_slicer_control`](ugv_slicer_control/) 和 [`slicer_wasd_control`](slicer_wasd_control/)。
- GNSS 差分看 [`cup_gnss`](cup_gnss/)，视觉抓取看 [`slicer_visual_grasp`](slicer_visual_grasp/)。
- 空地通信看 [`ugv_communication`](ugv_communication/)、[`slicer_air_ground_msgs`](slicer_air_ground_msgs/) 及 [V2 协议](ag_uav_client/docs/protocol_v2.md)。

本目录保留 ROS 包名和接口。底层硬件与第三方源码的版本和位置见 [dependencies.repos](../dependencies.repos)。
