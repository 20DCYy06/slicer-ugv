# 审查控制链修复 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox syntax for tracking.

**Goal:** 消除确定的 PWM 调用路径缺口与 Guard 动作改写，补齐可归因的轻量观测，并完成隔离软件验证。

**Architecture:** 在现有 ugv_slicer_control 内复用连续映射并安装受控原 driver 副本，独立显式 launch。Guard 直接拒绝不能执行的整条请求；保留既有 Mux/Watchdog，无新增运行管理层。

**Tech Stack:** ROS1 Noetic、rospy、diagnostic_msgs、Catkin、unittest/rostest、无设备且 network=none 的 Docker。

**Spec:** docs/superpowers/specs/2026-10-23-review-control-fixes.md；技术依据 docs/独立审查报告_Slicer_ROS1_2026-10-23.md。

## Global Constraints

- 旧项目和队友工程只读；本轮不部署 Jetson、不启动真实驱动、不 release、不发实车目标/速度。
- Guard raw→Guard→Mux→唯一 driver 的边界保留。诊断不另开串口，不能阻断停车。
- 物理参数仍为参考/暂定；不得将 TEST_ONLY 结果写成实车通过。
- 不改角速度、停车容差、GlobalPlanner/unknown/footprint，不引入 RTK、任务恢复或管理框架。

## Review Focus

- installed/devel driver 必须导入本包修复函数，而不是同名队友文件。
- 非有限/过期命令、遥测延迟和精确零保持原有停止语义。
- MCU ack、timer ticks 与主机发送序号/permille 的单位和语义不得混淆。
- Guard 不把负 vx 曲线改成另一条运动；合法纯转与取消零仍可立即传递。
- 原理想模型通过不代表驱动或地面响应；新整链模型不得积分 raw 旁路。

### Task 1: 源码基线与已知缺陷回归

**Files:** .gitignore、src/ugv_slicer_control/test/test_driver.py、test/test_navigation_guard.py。

- [x] 保存源码基线到 Git，日志/bag/镜像/审查导出不入库；建立 fix/review-control-chain 分支。
- [x] 运行现有隔离套件，记录准确结果；新增回归先运行得到预期失败。

### Task 2: 受控 driver、连续映射与诊断

**Files:** src/ugv_slicer_control/scripts/{c30d_driver_node,c30d_link,chassis_core,serial_transport}.py；CMakeLists.txt、package.xml、SOURCES.json；src/ugv_bringup/{config/slicer_driver.yaml,launch/slicer_driver.launch}。

**Interfaces:** 输入配置中 final Twist；输出原 wheel odom/ready、private diagnostic_msgs/DiagnosticArray；序号与 MCU tick 保留原语义。

- [x] 测试 actual main→wheel_pwm→Slew→Chassis.pwm→帧的连续/限幅/零/反向/超时行为，先 RED。
- [x] 只迁移必要通信模块并接现有连续函数，保留安全行为；逐帧诊断由唯一串口所有者输出。
- [x] 确定性 fake transport 测试 GREEN；不以测试第二份算法替代实际调用边界。
- [x] 安装与 devel 路径验证后提交。

### Task 3: Guard 拒绝不可执行动作

**Files:** src/ugv_slicer_control/scripts/{navigation_core,arc_velocity_guard}.py；test/test_navigation_guard.py、test/guard.test、test/test_guard_ros.py。

**Interfaces:** 负 vx 且 max_reverse=0→整条零+明确 hold_reason；原始纯转仍受原限幅/几何检查。

- [x] 写负 vx 带角速、微小负 vx、合法纯转、零及过期输入测试，先 RED。
- [x] 最小修改拒绝策略，禁止补偿覆盖拒绝，保留失效/扫掠检查。
- [x] 单元与真实节点测试 GREEN，提交。

### Task 4: 整链隔离验证及文档

**Files:** src/ugv_navigation/test/*guard*；src/ugv_navigation/test/test_odom_navigation.py；必要测试配置/CMake；docs/field、INTERFACES、MIGRATION、VALIDATION、README。

- [x] 真实 move_base/TEB→Guard→Mux→TEST_ONLY 非全向模型验证；观察 raw 与 final，明确模型不覆盖真实电机。
- [x] 保留不可达/拒绝结果，不把侧方失败掩盖成通过。
- [x] 修正 300‰“起转条件”、原 driver 调用链、模型“真实到点”、固件源码 coast/实际镜像边界。
- [x] 完整隔离构建/测试/install 与独立复核，记录软件通过/未验证硬件/后续最小采样，提交。

## 执行记录

源码基线 `23728e8`。Driver 20项unit、Guard 6项unit和11项ROS场景；最终完整套件73条目通过并安装。source/devel/install导入路径、真实ROS诊断mock传输另验。首轮完整套件1失败、夹具屏障首版开发失败和环境TF回拨记录均保留，生产超时及断言未放宽。独立复核的startup晚订阅缺口已补provenance；observer措辞改为异常隔离，不声称硬实时无阻塞。详见`docs/field/REVIEW_FIX_VALIDATION.md`。

本轮选择整条拒绝禁止动作，未选择无证据调参；代价是侧方目标仍会中止。历史GlobalPlanner失败缺原始输入而不改配置，代价是该失败仍可能复现。实车起转、停车与运动定位需要同步数据，不能由理想模型替代。没有部署或ARM64新版本验收。
