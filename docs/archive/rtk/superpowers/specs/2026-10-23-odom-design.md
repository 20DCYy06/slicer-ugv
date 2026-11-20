# 第一阶段 odom 导航设计

用户已经明确要求本轮直接完成第一阶段；按该授权实施，不新增设计审批或运维框架。

目标：建立独立 Noetic Catkin workspace，让用户和队友通过标准 move_base 接口测试局部滚动导航。

采用两个包：ugv_bringup 提供参数和 launch；ugv_navigation 提供 move_base 启动、直接 Action 客户端及隔离测试。既有 Slicer LIO/轮速 EKF 保持 /odom 和 odom→base_link 所有权，既有 /scan 必须可经真实 TF 转到 base_link/odom。本工程不发布生产 TF、启动驱动或接管硬件生命周期。定位独立测试通过 rostopic/tf2_ros；第二阶段有实际 RTK 实现后再创建 ugv_localization。

选择标准 move_base + GlobalPlanner + TEB；两个 costmap 都在 odom 中滚动，不依赖 map/AMCL/静态图，不发布 map→odom。禁止规划到 unknown，默认目标须在当前已观测窗口内。禁用旋转/清图 recovery；move_base 的原目标重规划可处理有限时间内的临时阻塞，ABORTED 不擅自重发。保留原 Guard/Mux/C30D 的急停、Watchdog 和接管锁存责任。

默认 ROS namespace ugv；odom/scan/cmd_vel 由 launch 参数指定。默认速度输出为 namespace 内 cmd_vel/nav_raw，实际接现有 Guard 时显式传入其 raw topic，且必须将外部 Guard 的 planner_namespace/costmap_topic 对齐。绝不提供直发底盘入口。

替代方案：整体搬 cup_nav_bridge 会保留 Session/Contract/Operations 耦合，违背用户目的；自写导航 Action/状态机会重复 move_base，第三阶段仅为必要恢复语义增加最小逻辑；本阶段均不选。

软件验证：本机已有 Noetic 镜像，network none、无设备、独立 master。真实 move_base/TEB/costmap 参与，测试专用运动学模型消费 raw Twist 并产生 odom/scan/TF。验证到点、取消不复活、障碍/unknown/传感器缺失不继续输出运动。模型不能证明定位精度、Guard/Mux/C30D、制动或避障硬件能力。所有失败原始输出保留。

后续：阶段2选择迁移 ENU/对齐/robot_localization 和定位有效性检查；阶段3迁移有界恢复和必要上层 Action。未知安装/标定参数不能编造。
