# 全局路径 B 样条平滑

## 实际数据流

`move_base -> BSplineGlobalPlanner::makePlan -> 原GlobalPlanner/Dijkstra -> 三次B样条 -> TEB::setPlan`

这是一个 `nav_core::BaseGlobalPlanner` 后处理插件。TEB不订阅一个话题来接收
导航参考线，因此不能靠重发 `/move_base/TebLocalPlannerROS/global_plan` 替换它。
插件真正返回平滑后的位姿数组；话题只用于诊断与RViz显示。

原有CLI、地图、AMCL、巡航位置锁定/最终yaw、底盘闭环与速度出口均保持原逻辑。

## 算法与边界

- 去除相邻重复点；RDP默认误差3cm，保留起终点与显著折角。
- 沿关键点多边形补充控制点，默认间距不超过20cm。
- 使用三次、端点重数4、内部简单均匀节点的夹持B样条，无SciPy依赖。
- 用解析一阶导数计算内部点航向，弧长表反查u，约每5cm取一个样条点。
- 在全局代价地图快照中细采样检查中心代价与现有车身足迹，不新增车体边距。
- 默认接受中心代价0..252，拒绝内切到253/254/255。不是把所有膨胀层都当墙。
- 内切时缩小RDP误差与控制点间距，最多细化两次；仍不安全就完整返回原路径。
  不把局部折线与样条拼接，避免人为引入新的接缝不连续。
- 底盘最小转弯半径仍为0，平滑器不发布速度、不增加PWM/速度限制。

正常、无零导数的样条几何曲线内部为C2，曲率连续。`nav_msgs/Path`传输的是离散采样点，
TEB还会重新离散和优化，因此不能声称TEB执行轨迹严格C2、时间上的加加速度连续，
或不同次重规划的路径之间严格连续。

起点与原规划器有效终点坐标严格保留；起点实际朝向和终点任务yaw也保留。
这两个姿态是任务边界条件，可能与几何切线不同；内部点航向才使用样条解析切线。
碰撞回退的原网格路径也不承诺C2。`smoothing_status`会明确显示是否发生回退。

## 配置

`config/navigation_params.yaml`在生成的车辆参数之后加载：

```yaml
base_global_planner: slicer_navigation/BSplineGlobalPlanner
GlobalPlanner:
  smoothing:
    enabled: true
    rdp_epsilon: 0.03
    control_point_spacing: 0.20
    sample_spacing: 0.05
    max_refinements: 2
    max_cost: 252
```

原`GlobalPlanner`参数原样复制给内部`GlobalPlannerRaw`实例。
TEB配合调整为时间权重1.5、最短路径权重0.2、角速度/角加速度权重各1.0，
开启振荡恢复，v/omega检测epsilon各0.1。转弯半径权重1.0在最小半径0时不引入非零半径限制。

仅关闭平滑：将 `GlobalPlanner.smoothing.enabled` 改为false，随后重新启动导航。
完全切回原插件：将 `base_global_planner` 改为 `global_planner/GlobalPlanner`。
本次没有改动生成器，生成参数仍提供原插件默认值，上述算法配置覆盖该值。

## 编译、检查、观测

```bash
cd /home/potato/Slicer_Workspace
NAV=./catkin_ws/src/slicer_navigation/scripts/run_navigation.sh
"$NAV" build
"$NAV" check
```

`check`在无网络、无串口/传感器设备的Noetic容器中完成：
1. 几何检查：端点、内部节点C2、弧长采样、内切细化、不可行回退和退化路径。
2. 原有导航检查，并加载真实插件，在合成障碍图上运行真实GlobalPlanner与TEB。
3. 检查`/move_base/make_plan`返回的是平滑结果，而不仅是显示话题。

实车由操作者按原CLI启动；本次离线验收不会启动硬件或发出真实导航目标。
RViz用两个Path显示对比：

- 原始网格：`/move_base/GlobalPlanner/plan_raw`
- 交给TEB的结果：`/move_base/GlobalPlanner/plan`（保持原RViz话题）
- 状态：`/move_base/GlobalPlanner/smoothing_status`

```bash
docker exec -it slicer-navigation bash -lc \
 'source /opt/ros/noetic/setup.bash; rostopic echo /move_base/GlobalPlanner/smoothing_status'
```

`smoothed raw=... output=... refinement=...`表示采用样条；`raw fallback: ...`表示保留原路径。
内部GlobalPlanner也会发布`/move_base/GlobalPlannerRaw/plan`，用于核对底层搜索结果。
