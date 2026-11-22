# 裸机 Noetic 依赖

目标环境是 Ubuntu 20.04 Focal / ROS1 Noetic。旧 SSD 宿主是 Ubuntu 22.04.5 Jammy、AArch64、L4T R36.4.3，内核为 5.15.148-tegra。宿主安装了 Humble，Noetic 由容器提供。R36.4.3 对应 JetPack 6.2 系列，盘上没有 JetPack 元包安装记录。[NVIDIA 版本对照](https://developer.nvidia.com/embedded/jetpack-archive)

| 组件 | 原生选版及证据 |
|---|---|
| ROS Noetic 和导航 | 各包的 package.xml 提供 rosdep 输入，另核对 CMake 使用的 Boost、Eigen3、PCL、APR、Qt5 Widgets、Python 开发头和 JSONCPP/OpenSSL。现有导航使用 move_base、GlobalPlanner、DWA、TEB、Gmapping、robot_localization、点云转扫描和 RViz。保留最终可用软件源与通过验证的包版本。[ROS 官方说明](https://discourse.ros.org/t/ros-noetic-end-of-life-may-31-2025/43160) |
| Livox SDK2 | Focal 重建固定到 `c0796f04c143143899c87a773d9f6b7136453c0b`（1.5.2）。旧宿主三个头文件与该提交逐字节相同，宿主共享库也报告 1.5.2。旧库为 AArch64、使用 Jammy ABI，需在目标环境重新编译。旧 Noetic 镜像层内版本和本地补丁仍未确认。[SDK 源码提交](https://github.com/Livox-SDK/Livox-SDK2/commit/c0796f04c143143899c87a773d9f6b7136453c0b) |
| FKIE | Noetic 发布版 1.3.2-2 为重建候选，提供 fkie_master_discovery、fkie_master_sync、fkie_multimaster_msgs。旧 Dockerfile 检查基镜像中已有的可执行文件，较早配方通过 apt 安装。镜像中的确切版本和层内补丁未确认。[FKIE 发布仓库](https://github.com/fkie-release/multimaster_fkie-release) |
| GNSS | 既有部署验证用 nmea_navsat_driver 0.6.1、ntrip_client 1.4.1 和 pyserial，由 cup_gnss 负责接收机串口。RTK 站点和杆臂见 gnss_site.yaml。 |
| 底盘与固件 | 主机设备枚举用 pyserial、fuser（psmisc）。旧盘 v1.4 ELF 的 GCC 标记为 10.3.1。旧包记录 gcc-arm-none-eabi 15:10.3-2021.07-4、binutils-arm-none-eabi 2.38-3ubuntu1+15build1 和 libnewlib-arm-none-eabi 3.3.0-1.3。重建固件要对照工具链和原镜像哈希。 |
| 视觉和本地交互 | 视觉包通过 rosdep 声明 NumPy、OpenCV、Pillow、ImageTk、Tk、pyserial、PyYAML 和 psmisc。Focal 的 ImageTk 软件包名为 `python3-pil.imagetk`。运行图形界面使用车机已登录的桌面。独立硬件工具用系统 Python，手柄读 Linux 输入设备。 |

本轮用 Ubuntu Base 20.04.5 amd64、Focal `main universe` 与 Noetic 已签名的 `noetic/final/ubuntu` 快照完成构建。实装 Catkin 0.8.12，move_base/GlobalPlanner/DWA 1.17.3，TEB 0.9.1，Gmapping 1.4.2，robot_localization 2.7.7，pointcloud_to_laserscan 1.4.1，FKIE 1.3.2-2。CMake 为 3.16.3；SDK2 按下方提交构建。APT 完整版本及日志留存在本地迁移记录。

## 固定源码依赖

[dependencies.repos](../dependencies.repos) 是工作区源码版本清单。用 Focal 的 `python3-vcstool` 执行 `vcs import . < src/slicer-ugv/dependencies.repos`。工作区独立获取 FAST-LIO、Livox ROS Driver2、硬件仓和 Livox SDK2。

- FAST-LIO 的团队分支保留上游历史和已用整车实现。对应上游提交为 `7cc4175de6f8ba2edf34bab02a42195b141027e9`，内嵌 ikd-Tree 与其原 gitlink 对应的 `e2e3f4e9d3b95a9e66b1ba83dc98d4a05ed8a3c4` 一致。运行输出由 `~runtime_dir`、`SLICER_RUNTIME_DIR/fast_lio` 或 ROS_HOME 下的 `fast_lio` 目录决定。[团队 fork](https://github.com/20DCYy06/FAST_LIO)
- Livox ROS Driver2 的上游版本为 1.2.8，基准 `21445540f0d100dc86a7e6df312dd70bbdb4afdf`。构建脚本把固定 SDK2 编译到 `W/sdk/install`，将头文件和静态库通过 `LIVOX_SDK2_ROOT` 交给驱动。安装检查核对具体编译与链接路径。[团队 fork](https://github.com/20DCYy06/livox_ros_driver2)
- 既有 Noetic APT/rosdep 包继续由系统提供。发布保存实装软件包版本。Livox SDK2 直接由构建脚本在工作区编译。

## 授权及维护待确认项

- FAST-LIO 根许可证为 GPL-2.0，包清单声明 BSD，部分文件带 BSD 式声明。fork 保留原文及作者。由维护者确认各文件的授权适用范围。
- 整车集成的 `ag_uav_client` 和 `ag_ugv_task` 声明 `Proprietary`，`slicer_air_ground_msgs` 声明 BSD-3-Clause。与空地团队确认分发许可和消息定义归属。
- `ugv_slicer_control` 同时声明 BSD-3-Clause 和 MIT，文件来源见其 `SOURCES.json`。维护者可补充逐文件授权和有效联系人。现有占位维护者邮箱保留。
- 硬件仓保留 ARM CMSIS、ST 固件库及已有项目归属声明。重新发布第三方固件源码前核对各自文件中的授权。

## 设备和现场配置

现场底盘输入可单独保存为 `~/workspace/slicer/site/hardware.yaml`。构建及预检时显式设置 `SLICER_HARDWARE_CONFIG`，默认使用固定硬件仓的 `config.yaml`。完整安装将所选配置写到对应的 RTK/NoRTK 资源及构建记录。修改硬件输入后重新生成候选并验收。GNSS 凭据及视觉标定仍按既有外部覆盖规则管理。

- 首次配置 rosdep 的 Focal 目标环境，安装 `build-essential cmake python3-rosdep python3-vcstool ros-noetic-ros-base`，执行一次 `sudo rosdep init`，随后按 README 更新及解析依赖。
- 若访问 GitHub Raw 时连接被重置，可按镜像站说明同时配置 rosdep 源列表和 `ROSDISTRO_INDEX_URL`，继续以普通用户运行 `rosdep update --include-eol-distros --rosdistro noetic`。[中科大 rosdep 镜像说明](https://mirrors.ustc.edu.cn/help/rosdistro.html)
- 按设备枚举确认 `deploy/udev/99-c30d.rules` 与 `80-cup-gnss.rules` 的实际匹配项。安装匹配的规则后核对 C30D 和 GNSS 的设备别名及 dialout 权限。
- 复制安装后的 `slicer_gnss.yaml` 到 `~/.config/slicer/gnss.yaml`，设置实际 serial_port、ntrip_host、ntrip_port、ntrip_mountpoint、ntrip_credentials_file。凭据由操作员提供，0600 JSON 包含 username 和 password，仅留车端。把外部 YAML 的绝对路径传给 `slicer_field_sensors.launch config_file:=...`。默认 YAML 里的 /dev/gnss 和 /run/secrets/ 对应旧容器。
- 启动前按现场网段核对 MID360、主机、空地电台和 CORS 配置。设备及站点记录保存在现场资料。

确认目标机使用对应设备后，从仓库根安装规则：

```bash
sudo install -m 644 deploy/udev/{99-c30d,80-cup-gnss}.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
```

重新连接设备后检查 `/dev/c30d` 和 GNSS 的 by-id 路径。若刚加入 `dialout`，重新登录后再核对权限。

安装后依 [README](../README.md) 运行源码预检、构建和安装检查，在任务交接中记录 APT、SDK、FKIE 和生效的设备配置。
