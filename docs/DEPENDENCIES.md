# 裸机 Noetic 依赖

目标环境是 Ubuntu 20.04 Focal / ROS1 Noetic。旧 SSD 宿主是 Ubuntu 22.04.5 Jammy、AArch64、L4T R36.4.3，内核为 5.15.148-tegra。宿主安装了 Humble，Noetic 由容器提供。R36.4.3 对应 JetPack 6.2 系列，盘上没有 JetPack 元包安装记录。[NVIDIA 版本对照](https://developer.nvidia.com/embedded/jetpack-archive)

| 组件 | 原生选版及证据 |
|---|---|
| ROS Noetic 和导航 | 各包的 package.xml 提供 rosdep 输入，另核对 CMake 使用的 Boost、Eigen3、PCL、APR、Qt5 Widgets、Python 开发头和 JSONCPP/OpenSSL。现有导航使用 move_base、GlobalPlanner、DWA、TEB、Gmapping、robot_localization、点云转扫描和 RViz。保留最终可用软件源与通过验证的包版本。[ROS 官方说明](https://discourse.ros.org/t/ros-noetic-end-of-life-may-31-2025/43160) |
| Livox SDK2 | Focal 重建固定到 `c0796f04c143143899c87a773d9f6b7136453c0b`（1.5.2）。旧宿主三个头文件与该提交逐字节相同，宿主共享库也报告 1.5.2。旧库为 AArch64、使用 Jammy ABI，需在目标环境重新编译。旧 Noetic 镜像层内版本和本地补丁仍未确认。[SDK 源码提交](https://github.com/Livox-SDK/Livox-SDK2/commit/c0796f04c143143899c87a773d9f6b7136453c0b) |
| FKIE | Noetic 发布版 1.3.2-2 为重建候选，提供 fkie_master_discovery、fkie_master_sync、fkie_multimaster_msgs。旧 Dockerfile 检查基镜像中已有的可执行文件，较早配方通过 apt 安装。镜像中的确切版本和层内补丁未确认。[FKIE 发布仓库](https://github.com/fkie-release/multimaster_fkie-release) |
| GNSS | 既有部署验证用 nmea_navsat_driver 0.6.1、ntrip_client 1.4.1 和 pyserial，由 cup_gnss 负责接收机串口。RTK 站点和杆臂见 gnss_site.yaml。 |
| 底盘与固件 | 主机设备枚举用 pyserial、fuser（psmisc）。旧盘 v1.4 ELF 的 GCC 标记为 10.3.1。旧包记录 gcc-arm-none-eabi 15:10.3-2021.07-4、binutils-arm-none-eabi 2.38-3ubuntu1+15build1 和 libnewlib-arm-none-eabi 3.3.0-1.3。重建固件要对照工具链和原镜像哈希。 |

本轮用 Ubuntu Base 20.04.5 amd64、Focal `main universe` 与 Noetic 已签名的 `noetic/final/ubuntu` 快照完成构建。实装 Catkin 0.8.12，move_base/GlobalPlanner/DWA 1.17.3，TEB 0.9.1，Gmapping 1.4.2，robot_localization 2.7.7，pointcloud_to_laserscan 1.4.1，FKIE 1.3.2-2。CMake 为 3.16.3；SDK2 按下方提交构建。APT 完整版本及日志留存在本地迁移记录。

在目标 Focal 环境构建 Livox SDK2。Catkin 在 /usr/local/lib 查找 liblivox_lidar_sdk_static.a。

```bash
git clone https://github.com/Livox-SDK/Livox-SDK2.git
git -C Livox-SDK2 checkout c0796f04c143143899c87a773d9f6b7136453c0b
cmake -S Livox-SDK2 -B Livox-SDK2-build -DCMAKE_BUILD_TYPE=Release
cmake --build Livox-SDK2-build -j2
sudo cmake --install Livox-SDK2-build
```

## 设备和现场配置

- C30D 使用 `deploy/udev/99-c30d.rules` 的板号 5CF0016129。旧系统 /etc 同名规则是较旧的板。恢复的 `80-cup-gnss.rules` 匹配接收机 1a86:55d3、序列号 5185028488，别名为 `/dev/serial/by-id/usb-1a86_USB_Single_Serial_5185028488-if00`。安装匹配的规则后核对别名和 dialout 权限。
- 复制安装后的 `slicer_gnss.yaml` 到 `~/.config/slicer/gnss.yaml`，设置实际 serial_port、ntrip_host、ntrip_port、ntrip_mountpoint、ntrip_credentials_file。凭据由操作员提供，0600 JSON 包含 username 和 password，仅留车端。把外部 YAML 的绝对路径传给 `slicer_field_sensors.launch config_file:=...`。默认 YAML 里的 /dev/gnss 和 /run/secrets/ 对应旧容器。
- 旧 MID360 雷达/主机地址是 192.168.1.192 / 192.168.1.5，旧接口 enP8p1s0。电台车端/飞机端为 192.168.10.33 / 192.168.10.32。在新系统按实际接口确认。旧盘两种 CORS 配置都关联临时服务，当前服务和站点基准由现场确认。

安装后依 [README](../README.md) 运行源码预检、构建和安装检查，在任务交接中记录 APT、SDK、FKIE 和生效的设备配置。
