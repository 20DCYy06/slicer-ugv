# RTK 导航稳定性与地图覆盖（2026-11-25）

## 基线与交付顺序

主线基线 `93ccd8c82fe6af332c37692ec34510baeb0f70c3`，任务分支 `fix/rtk-navigation-stability`，工作区 `~/workspace/slicer/dev/rtk-navigation-stability`。交付优先处理路线连续性、二维地图、桌面和配准；VoxelLayer 从该修复分支延伸，使用独立任务和发布路径。正式 `current` 核对为 `releases/20261123-rtk-fixed-a7a64c7`。历史现场记录来自 `releases/20261124-rtk-navigation`。候选在原位置编译，切换由现场负责人协调。

2026-11-25 花坛袋包为 `~/workspace/slicer/data/rtk-corner-candidate-20261125-051745/motion.bag`，同目录 `analysis/RTK_NAV_ACCEPTANCE_REPORT_20261125.md` 保存现场分析。新鲜 Fixed 为 2091/2091，局部净位移约 0.49 m，累计绝对转角约 298°，约 60.7 秒后以 oscillation 停止。raw 和 final 路线在车近乎静止时切换绕行侧。袋中有二维栅格和扫描，高度遮挡与可视范围需要另采单帧点云。

## 参数与现场入口

二维默认使用固定 `map` 网格 90×90 m、5 cm、原点 (-45,-45)，局部仍是 `odom` 下 9×9 m 滚动图。启动时从 `ugv_bringup/config/outdoor_map.yaml` 读取范围；自定义范围可设 `SLICER_RTK_OUTDOOR_MAP_CONFIG` 并停车重启。配准地图预览给出实际边界及当前目标是否在图内。居中范围足以覆盖 ENU 原点位于或紧邻任意朝向的 30×30 m 测试场；真实可通行边界由现场负责。

空投影角格保持未观测；两张 RTK costmap 使用有限回波清除。扫描仍经过原有自体过滤；无 RTK 配置独立。基于观测范围和停车预算检查可执行近段，继续按原 footprint 和净空门槛拒绝不安全路线。现有控制链和 TF 所有权沿用。

`start_heading_enabled` 默认为 true，正式入口启动时用 `SLICER_START_HEADING_ENABLED=false` 进行单变量对照。桌面配准支持从已观测直线走廊选取最长可行的前进段（默认上限 2.0 m），保持原有直线返程和配准质量门槛。返程停稳与 Mux 实例确认后进入导航地图。**WGS84 测试目标**和 **Waypoint 打点**走同一目标提交链。

参见[操作手册](../OPERATIONS.md#rtk-导航稳定性候选)与[正式切换与回退](../MAINTENANCE.md#候选发布与切换)。

## 阶段状态

主线地图与开关变更、桌面离线回归已完成；路径连续性和配准集成正在验证。候选发布证据会追加到本节。实车绕障、初始转向单变量对照及实测三维回波比较待现场验收。
