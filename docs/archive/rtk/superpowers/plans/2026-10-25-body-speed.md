# 0x16车体IMU接口最小接入 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 用户已选择并授权继续接入队友现有0x16车体IMU闭环，保持原导航/Guard/Mux/唯一driver/Watchdog。

**Architecture:** 在现有Chassis中增加0x17/0x84能力查询、0x16车体目标、0x93反馈解析，沿用唯一串口和原0x90诊断。现有driver增加显式body_speed分支，继续发布原编码器wheel/odom，不修改融合或TF。默认模式不变，以单独配置选择body_speed。

**Tech Stack:** ROS1 Noetic、Python、现有SLC2帧/CRC/nonce、unittest与断网无设备容器。

**Spec:** 本页记录刚才用户“好，继续”批准的小范围方案；现行队友参考在`.test-results/body-speed-20261025/reference/`，来源和SHA另存。0x16=<Ihh>(nonce,mm/s,mrad/s)，0x84=<IBBBB>，0x93为64字节version1。

## Global Constraints

- 队友树/旧项目只读；不改MCU、规划器、Guard/Mux行为或融合。
- 不启动真实driver、接串口、release/hold或发送目标/速度；运动由用户执行。
- 仅更新独立工程；ARM64构建使用--network none且无设备。
- 不自动启用新模式，不以PWM/0x10回退掩盖body模式失败。

## Review Focus

- nonce不匹配/扩展不支持：拒绝body模式，保留同模式零速关闭。
- 0x90持续但0x93丢失、重复时间戳或IMU失效：非零出口归零，不能被其他反馈保活。
- 零目标、主机命令超时、故障/串口异常及退出：发送0x16四舍五入后的车体双零，不走旧模式回退。
- SI单位转mm/s、mrad/s；反馈单位、CRC/序号、原wheel/odom不变；body记录不能被分析器漏判。
- 原PWM/轮速、Guard/Mux测试及安装入口继续有效；新配置不改原默认。

## Tasks

- [x] 1. 协议：先增加实际Chassis编解码/匿名管道失败测试，再实现body字段、能力握手、独立反馈时钟及同模式关闭。文件`c30d_link.py`、新`test_body_link.py`。
- [x] 2. driver：先扩展现有真实driver管道测试，验证body包/不回退/IMU失效停零等失败；再修改`control_core.py`、`c30d_driver_node.py`。配置新增`ugv_bringup/config/slicer_driver_body_speed.yaml`，原配置不变。
- [x] 3. 消费者：先增加采集/分析对kind22和body反馈的失败用例，再适配既有tools；不新增记录系统。
- [x] 4. 运行相关单元回归、断网隔离ROS接口测试、Catkin安装解析；审阅最终差异。
- [x] 5. 按已有授权同步到Jetson独立工程，ARM64无设备构建/检查，不重启任何真实节点；提供一次底盘终端切换与用户纯转验收命令。

## Interfaces

- `Chassis.body_speed(vx_m_s,wz_rad_s)->seq`：0x16；serial_tx事件含`body_vx_mm_s`、`body_wz_mrad_s`。
- `Chassis.require_body_control(timeout=5.0)->(nonce,version,who,imu_flags,source)`：新nonce能力查询，校验version1/who0xea，并等待有效body反馈。
- `Chassis.body_feedback_fresh(max_age=.30,max_imu_age_ms=100)->bool`：0x93时间推进、IMU flags7、who0xea及报告age有效；不改普通last_rx。
- 属性`body_info`、`body_telemetry`、`last_body_rx`；body_telemetry诊断字段复用队友decode_body的键名与单位。
- command_tx事件body字段`target_body_vx_mm_s`、`target_body_wz_mrad_s`；原SI输入和通用诊断保留。

## Verification ledger

- 协议18项、driver66项（原50+新16）、配置4项均已RED→GREEN，失败日志保留。
- WSL控制包Catkin汇总130项记录，0错误/失败/跳过；最终工具68项在Noetic中通过、无跳过。
- 独立代码审查未发现生产driver阻断问题；已修复IMU failed位误判、body ACK后duty关联和连续新零序号对应问题，各有失败再通过的测试。
- 已核对参考body_update收到双零会reset外环积分并输出四零；未编译/修改或刷写该固件。
- ARM64安装及参数/哈希核对通过；车端再次130项控制测试记录、68项工具检查均通过，无失败/跳过。真实driver由用户切换，握手与纯转尚待现场执行。
