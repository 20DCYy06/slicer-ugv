# Slicer odom 最小实车接线计划

**Goal:** 新工程接现有Slicer定位和控制接口；本轮仅审查、ARM64构建、无运动目标验证，运动由用户现场执行。
**Architecture:** 既有move_base不变；launch配置Guard raw/namespace/costmap与Mux新release owner。队友当前源缺少这些参数，选择性复用此前已验证的Slicer Guard/Mux适配代码，独立包存放且不携带C30D驱动、不覆盖队友包。一个短生命周期客户端完成显式控制权交接，不新增后台管理器或任务状态机。
**Spec:** 本轮用户消息；只完成真实输入可规划、短距离到点、基础障碍响应的准备；不做RTK、自动恢复、部署系统优化。

## 约束与审查发现

- 旧工程和队友 `/home/potato/Slicer_Workspace` 只读；新增车端目录 `/home/potato/slicer_ugv_ros1`。
- 用户确认 `192.168.10.33` 为电台地址；不修改原通信网络配置。车端只读审查可显式用 localhost 访问同一master。
- 当前队友Mux无manual hold/授权参数，Guard硬编码/move_base；当前没有运行Slicer定位/Guard/Mux。当前选定681派生实现已有参数和代次绑定，不能把候选代码当正在运行的版本。
- 实车不解除保持，不发Action目标、不发非零速度。原C30D、Watchdog和手动接管保留。
- 隔离模型只在无设备network=none独立master运行，不能作为真实输入或实车验收。

## Task 1: 控制接线

- [x] `ugv_slicer_control`仅保留既有Guard/Mux及其3个纯逻辑依赖，按源文件记录hash，不更改既有策略。
- [x] 一次性`control.py`使用真实新身份`/ugv/control`，Mux通过`manual_release_owner`明确授权；读取当前实例/代次，校验匹配ACK，绝不冒充旧执行器。
- [x] 标准launch接 `/cmd_vel/nav_raw→Guard→/cmd_vel/nav→Mux→/cmd_vel`；不包含真实驱动，不提高已标注的调试速度。
- [x] 隔离测试启动held、新owner/旧owner、代次、急停、取消/hold等必要行为。

## Task 2: 真实输入与短目标

- [x] 既有goal客户端增加一次TF转换的相对目标和GetPlan入口；打印固定odom目标，拒绝缺失/过期输入，不跟随base移动。
- [x] 无目标验证用独立namespace/raw，检查真实odom/scan/TF和GetPlan；不将图中只有subscriber的`/odom`当真实发布数据。
- [x] ARM64原生构建、相关定向测试及launch展开；使用已有环境，不改驱动配置。

## Task 3: 现场短单

- [x] 记录真实输入缺项与可验证结论，保留失败证据。
- [x] 最短启动、授权、短目标、取消、接管与停车确认命令，注明仅用户现场执行运动。
- [x] 独立代码审查，实际软件通过与未完成的3项现场目标分开。
