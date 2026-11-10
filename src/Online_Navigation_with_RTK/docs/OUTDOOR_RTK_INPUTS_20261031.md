# 户外 RTK 实施约定与输入表

承接0423b8e，保留开始时4个dirty文件。用户附件是本轮执行规格；只在现有工作树离线实现，不接车/部署。采用既有cup_gnss ENU数学加Noetic robot_localization全局EKF，不采用UTM/navsat_transform，也不另写滤波器。

| 输入/输出 | frame、分量及时间 | 协方差、relative/differential、杆臂 |
| --- | --- | --- |
| 本地 `/odom` → `/geographic/motion` | child=base_link，仅vx、vy、wz；保留原header | 原局部融合twist covariance；false/false；不再融合其LIO/轮速原输入 |
| `/gnss/fix` + 同stamp `/gnss/fix_quality` | 天线原始WGS84经纬度/椭球高；主机接收适配stamp，非PPS测量时间 | raw q4且fresh_rtk_usable与fix_valid；未知covariance显式0.5m标准差假设；不是厘米精度 |
| `/gnss/odometry` | 固定ENU对齐map中的base_link x/y，同一fix stamp | 仅x/y，false/false；以对应姿态减一次水平杆臂，叠加初始化角度不确定性 |
| `/global/odom` | 全局EKF map→base_link估计 | 全局EKF独占动态map→odom；本地EKF继续独占odom→base_link |

配置复用gnss_site.yaml。天线位于雷达正上方：已有antenna_in_base_link_m水平值优先，空水平值从既有source_config的sensors.base_to_laser.xyz取得；不新增第二份尺寸。二维只使用x/y，高度不伪装实测。LIO已有雷达/IMU→base刚体换算不重复应用。

初始化：持久的geographic定位节点随field_sensors启动；先用field_navigation geographic:=false在odom下执行一次有限前进，配对同stamp原始RTK与插值odom，首尾停稳窗口拟合旋转/平移。收到明确begin请求才允许以首个合格点选原点，原点在本节点任务期固定；GNSS失锁/接收节点重连不重选。datum必须明确WGS84，未知时报告unconfigured。已有原点配置则直接使用。

配准后以SetPose初始化全局EKF，再接入运动twist及合格绝对GNSS。初始化前不转发全局滤波输入，因此不等待map TF即可做odom起步；结束后hold并重建导航实例，切回默认map入口。这也清除初始化/旧深度实例的障碍贡献。不是永久static TF。

短时失锁：保留原点/配准，继续局部运动积分，明确degraded；无合格RTK不确认地理到达。恢复超过3m创新的观测拒绝，保留原点并提示检查/重新配准；不向局部odom塞跳变。odom时间回退、frame变化或明显不连续使配准/地理活动目标失效，取消目标并复用control.py hold。无自动重新运动/重新配准。

默认global为map下40×40m固定实时雷达缓存，原点(-20,-20)，供5～10m首测，不代表完整30×40m任意朝向赛场；local为原4×4m odom滚动图。无StaticLayer、/map、D455依赖；全局允许未知规划，局部未知/越界和Guard保持原样。RTK不会重投影旧栅格；配准和明显累计map→odom修正后在hold下重建导航，不能因遇阻清图强行通过。

地理目标只用现有move_base Action。省略yaw采用当前位置到目标的方位，不暗中要求正东；显式yaw保留。原始GNSS的base参考点用于终点水平残差，与参与融合的RTK是同源核对，并非独立地面真值。

实施进度：A launch/Costmap离线探针，B纯几何，C全局桥与真实RL隔离联调，D现有Action/采集接入并行；root负责统一安装、回归、一次集中复核及交付。当前各项证据写入.test-results/outdoor-rtk-20261031。
