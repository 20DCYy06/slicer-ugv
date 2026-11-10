# 户外 RTK 候选：实际离线验证

> 本文为`878810d`阶段的历史离线验证记录，其中“尚未ARM/部署”和0.20档仅描述当时。后续`8e78b86`部署与0.4m/s配置见[部署记录](field/OUTDOOR_DEPLOYED_20261031.md)。下一次户外只使用[当前RTK操作单](field/OUTDOOR_RTK_20261031.md)；本轮F1/F2/capture版本354169a已完成ARM64部署与516项软件验证，户外实车结果仍待采集。

承接有效工作树`0423b8e`及开始时4个dirty文件，没有回退公开仓库或覆盖旧根目录。本轮本地x86_64、Noetic robot_localization **2.7.7**；不访问Jetson，不开真实串口/差分，不刷写/部署。本轮新代码未做ARM64构建，不能引用旧现场436项结果代替它。

## 已实现

- field默认D455/Gmapping关闭，不等`/map`，GlobalPlanner+DWA、0.20m/s、局部EKF、Guard/Mux/0x16不变。保留显式候选和原odom入口。
- 起步用停稳首尾的同时间RTK/odom轨迹拟合，保留非零初始odom yaw，水平天线杆臂只补偿一次。原点明确选定并保存，接收重连不重新选点。
- 全局RL只融合局部估计的body twist与合格绝对GNSS x/y，不重复融合原始组成输入。通过实际SetPose初始化，唯一动态map→odom；本地odom→base_link不被GNSS跳变覆盖。
- 每个fix新增同stamp的标准质量诊断，保留原1Hz诊断。raw q4、fresh_rtk与时效同时满足才参与；未知位置covariance用显式0.5m标准差假设，不当作零误差。
- 地理目标进入同一个move_base Action；支持经纬度与显式来源ENU、缺省朝向方位策略、原始GNSS/base终点残差及软件停稳分开报告。沿用既有采集器，不新建导航控制层。

完整输入/TF/基准/杆臂/时间约定见[输入表](OUTDOOR_RTK_INPUTS_20261031.md)，顺序命令见[现场短单](field/OUTDOOR_RTK_20261031.md)。

## 实际通过的检查

| 验证 | 实际结果 |
| --- | --- |
| Catkin构建、安装、软件回归 | **188 tools +317 Catkin =505项，0失败/错误/跳过**；干净install脚本/辅助模块、配置、外部包发现及launch展开通过 |
| 配准确定几何 | 东西南北、正负转角、非零初始yaw、杆臂、时间插值/不重合、静止/不足位移、不同ENU原点转换 |
| 真实RL联调 | 合成初始odom yaw=+π/2，求得旋转−π/2、平移[−4,3]；2m样本位移。合成零残差不代表实车精度，初始化角度仍保留5°假设下限 |
| 持续纠偏 | 注入+0.4m GNSS修正，真实global pose/map→odom变化约+0.394m，本地TF平移和yaw不变 |
| 质量与异常 | q5/陈旧fix不进入融合；失锁降级原点不变、恢复ready；重置失效并取消目标；同一动态TF边只有一个发布者 |
| reset的既有控制链 | 真实软件Mux确认takeover/manual_hold，generation0→1；测试输出599帧四零、0非零、0次release。无底盘/MCU，不代表物理停止；fixture无move_base状态服务器，记录了“held confirmed; cancel unconfirmed”的状态ACK缺口 |
| Action客户端 | 真实ROS传输下plan不发Action；到点复核、generation取消、非零初始yaw起步几何通过；使用合成服务器，不冒充真实路径跟踪 |
| 深度退出 | 深度开启占据100→停止/默认重建后0；深度参数消失，无关参数保留；雷达仍标记100/清除0 |

真实RL测试记录了请求时刻TF不可用、回退latest的警告；适配层自身仍要求有时间括住的odom/global样本才应用GNSS创新检查。本轮未测物理时间偏差或PPS同步精度。

集中复核修正了开发入口导入、全局预测迟到时跳过创新上限、会话落盘失败阻断取消/hold、重复reset等具体问题；保留先失败后通过的边界测试，不通过放宽阈值掩盖。ROS测试只使用无设备、`--network none`私有master。

## 6秒停顿定点对照

既有室内报告的120×120m、0.05m全局图有约6.20s消息间隔与约5.9964s更新日志；不是本轮重新归因为底盘/Guard。新对照使用实际move_base/GlobalPlanner、同一雷达软件输入、depth-off，配置4Hz更新/发布，无运动目标：

| 全局图 | 可达GetPlan最大 | 被围目标最大/结果 | 越界目标最大/结果 | 最大header间隔 |
| --- | --- | --- | --- | --- |
| 40×40m，800²格 | 0.01058s | 0.01704s／无路径 | 0.01049s／无路径 | 0.50009s |
| 120×120m，2400²格 | 0.09382s | 0.18114s／无路径 | 0.02105s／无路径 | 0.50026s |

每类3次，可达均34路径点；callback最大间隔分别约0.751/0.525s。这里的“被围”是实际Costmap中的合成闭合障碍，非实物测试。**本机未复现6秒停顿**，未做Jetson负载、无线链路或mutex锁等待归因；不能单凭缩图宣称根因已解决，也未放宽planner/oscillation超时。

## 复跑及范围

```bash
UGV_NOETIC_IMAGE=ugv-online-offline:20261030 ./tools/test_noetic.sh
UGV_NOETIC_IMAGE=ugv-online-offline:20261030 ./tools/noetic.sh python3 tools/check_geographic_localization_ros.py
UGV_NOETIC_IMAGE=ugv-online-offline:20261030 ./tools/noetic.sh python3 tools/check_geographic_client_ros.py
UGV_NOETIC_IMAGE=ugv-online-offline:20261030 ./tools/noetic.sh python3 tools/probe_outdoor_costmap.py --output .test-results/outdoor-costmap-repeat/probe.json
```

前两份probe是合成输入，使用真实Noetic节点/协议；没有合格户外RTK实测轨迹可供本轮回放，不能生成假实车数据。原始室内bag仍留在用户指定indoor-20261031-01目录；本交付只附相关配置/精选结果及路径，不重打大型bag。

后续只需：在车端独立目录完成本候选ARM构建；保留已有接收/安装配置，确认当前基准及通信；按短单完成一次起步配准、5～10m标记点和不同初始朝向复测，再雷达可见障碍绕行及失锁恢复。记录尺测落点/停车距离和Jetson的更新/规划时延。已有天线配置与RTK接收源码均已承接，不列为“尚未取得”；低矮/贴近障碍覆盖、物理到点/停止精度仍需现场验证。
