# ROS 布局整理交接 · 2026-11-20

- 上一轮基线为 `c80b68b436ec18ff4a31bb625f99a007239cccdf`。私有仓库的 `migration/native-noetic` 先保存此基线，再合入 `refactor/ros-layout`。默认分支用于迁移候选。
- 13 个 RTK、无 RTK、V2 活动包和 `ugv_communication` 物理移到 `src/<包名>`。Livox 保留上游文件和许可证，活动 manifest 是 ROS1；V1/V2 共用一份 `slicer_air_ground_msgs`。
- 正式视觉伺服从 `Test/Visual_Grasp` 包装成第 15 个包 `slicer_visual_grasp`，保留 UI、算法与设备实现。安装复用 `Hardware/Host` 与 AirGrip 支持文件，用户校准保存至 `$XDG_CONFIG_HOME/slicer_visual_grasp/config.json`，默认配置根为 `~/.config`。旧 `Vision_adjust` 见[历史入口](../archive/VISION_ADJUST.md)。
- 底盘遥控、舵机、气夹及反馈工具归入 `tools/manual/` 和 `tools/hardware/`，解锁留在 `Hardware/Unlock/`。组合手柄六份文件以旧 SSD 的 `Slicer_Workspace2/Test/tests/` 为来源，与指定 11 月 20 日快照原文件逐一核对。迁移保留 JSON、专用硬件参数和按键，按新位置调整代码与资源路径。
- `src/ugv_bringup/config/no_rtk/` 和 `Hardware/config.yaml` 是 NoRTK 生成输入，NoRTK runtime 与 RTK 生成输出随外部构建工作区。原生入口及旧命令对照见[操作手册](../OPERATIONS.md)。V1 FKIE 使用单一可解析的 launch，旧通信、RTK、NoRTK 和 V2 容器材料标为历史参考。

## 本轮验证

隔离 Ubuntu Base 20.04.5 amd64 / Noetic 使用提交 `eaf24a117ddc0171a46ad77db9a2146868311398` 的 Git 归档。只读源码、外部新建工作区、网络命名空间仅 loopback、无真实设备。后续交付提交更新文档和静态 CI 入口。

| 检查 | 结果 |
|---|---|
| `tools/build_native.sh`、`tools/check_native.sh` | 15 个包构建与安装成功。8 组 launch 已解析，FAST-LIO/Livox 安装件和 V1 消息 MD5 已核验。 |
| 安装后的视觉 | `rosrun slicer_visual_grasp run.sh --check` 验证 Python 导入与配置定位。离线测试 40 通过，1 项原始录制帧回放因缺样本而跳过。 |
| 空地 | V1 安装入口、辅助模块和消息序列化通过。客户端及路线 56 个实际用例通过，包含 22 个模拟 ROS 接收端用例。 |
| 独立工具 | 键盘手柄、融合手柄、AirGrip、PWM/HID、反馈及解锁 52 项纯软件测试通过。通信配置 3 项通过。 |
| NoRTK 与 RTK 参数 | NoRTK 8 份输入及 9 份生成文件与基线逐字节相符，RTK 11 份生成文件相符。替身 roslaunch 验证后台启停、重复启动及跨工作区 PID 隔离。 |
| 现有导航回归 | NoRTK B 样条几何与 9 个导航单元用例通过，RTK 定向选取 266 个导航及控制单元用例通过。GNSS 预览另有一条既有断言失败，原版本可复现。测试找 `datum` 错误，默认文件先报缺少 `map_from_enu`。 |

裸机 ARM64、真实设备、双机 FKIE、导航和视觉现场验收见 [主交接](../../HANDOFF.md) 的三项待办。
