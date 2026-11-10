# 工作边界

本目录是独立 ROS1 Noetic Catkin workspace（根目录直接运行 catkin_make）。
旧项目 `/home/rescue/industrial_cup_ros1` 只读，不修改、删除、部署覆盖。
2026-10-24 用户已明确取消禁止部署限制：允许向 Jetson 的独立新工程同步修复、做 ARM64 构建、接线审查和只读验证；不重复基础底盘运动测试。保持队友树与旧项目只读。助手不启动真实底盘驱动、不解除实车控制保持、不发实车目标或非零速度；驱动启动、release/hold 和运动由用户现场执行。允许保存与分析本次现场试验明确采集的日志和 bag，不扫描历史归档。
软件模型仅在无设备、`--network none` 的容器独立 master 中运行，不能当实车结果。
不扫描或复制 backup*、历史封存输入、日志、bag、凭据或生成缓存。
正常接口使用标准 ROS Topic/Service/Action；参数放 `ugv_bringup/config`。
速度只允许对接 Slicer Guard 的 raw 入口，绝不能直连底盘 `/cmd_vel`。
