# 完整修复与现场闭环

用户要求继续完整修复，车辆已上电、已换足电电池且本人在旁。沿用旧项目/队友树只读边界；新工程可更新与ARM64构建，真实driver、release/hold和运动目标只由用户现场操作。模型只在无设备、network=none容器运行。本计划不扩展RTK、自动任务恢复或部署管理系统。

## 当前证据与实施选择

- 保留上一轮基线`23728e8`和修复`ae4ced2/c467c94`。当前分支`fix/forward-field-navigation`，`1ea71ff`补有效配置、driver实例/诊断事件序号及明确单设备Docker映射。它们用于发现取证缺口，不构成运动门禁。
- 已核对39个Jetson源码/工具与保存基线一致，才只更新独立新工程src/tools。`1ea71ff`原生ARM64隔离构建、77个JUnit条目和安装通过；没有替用户启动硬件。
- 用户启动定位和新driver；新driver真实导入安装目录连续映射，mock=false、ready新鲜，Mux startup held、三段速度零、轮速与MCU回报零。当前C30D为`/dev/ttyCH343USB1`，身份`1a86:55d4 / 5CF0016377`。设备名不能跨重启假定。
- TEB五个fresh隔离实验已排除目标顺序、终态朝向、少两次迭代及重写全局朝向；起始需要侧转时仍为毫米级倒车局部解。初始已对准路径的对照成功。继续维护TEB需要修改初始化/剪枝/可执行性，优先验证标准DWA，保持公共Action、Costmap和安全链。
- DWA默认角轨迹分辨率.1在低角速短目标只采到初始姿态，前视点单参实验仍失败；仅改角分辨率为.025，7组目标加短侧目标重复共8/8成功。按实际DWA树接Guard后再验+.2成功；生产Slicer选择标准DWA，通用odom仍TEB作参考，保留失败与源码因果证据。

## 执行清单

- [x] 补实际有效配置、driver实例与递增诊断序号；24项driver unit RED→GREEN。
- [x] 在现有环境shell增加显式单设备映射；7项fake-Docker参数测试RED→GREEN。默认仍无设备、只进shell。
- [x] 初版ARM64隔离构建/77条目/安装；保留Docker bridge raw表缺失的镜像构建失败，改为仅依赖构建host网络，模型仍network none。
- [x] 真实输入、唯一接线、新driver provenance和held/零输出检查；前方0.5m GetPlan返回22点，无Action。
- [ ] 用户执行一次TEB+连续映射0.5m目标，记录命令→发送→接受/遥测→轮速/定位→地面位移；按同一goal归因，不将最后零或ack称作物理停车证明。
- [ ] 有因果依据地解决标准DWA短距侧转边界，复验正负侧方/斜前/纯转、取消、传感器失效、墙体与有限障碍。未通过不改默认。
- [x] 选定标准DWA后，Guard从实际插件参数读取限制，不借用未运行TEB的参数树；保持全链、unknown/外廓和现速度上限。本地完整86个JUnit条目、构建和安装通过。
- [ ] 两端完整回归、安装/namespace检查及只读实车GetPlan；再由用户进行新版本短距离、转向和基础障碍响应，必要校准须由同步数据支持。
- [ ] Git提交、更新现场短单和准确验证记录。未解决项如实列出，禁止将TEST_ONLY结果写成实车通过。

## 现场启动冲突保留

用户在新driver已运行后误启动原`start_ros.sh`。串口独占拒绝旧进程，但同名ROS注册使新driver退出；当前Mux一直startup held、最终零。原录包保留为`evidence/2026-10-23-complete-straight/straight.bag`。用户只重开D，新实例经只读诊断确认；重试记录另存`straight-retry.bag`。不将这次启动冲突算作运动试验。

## 授权与现场重试更新

2026-10-24用户明确允许同步修复并删除禁止部署限制，AGENTS.md已更新；真实驱动启动、release/hold和目标仍由用户执行。录包观察者使原客户端唯一订阅者检查阻止release/hold，unit和真实ROS均重现；最小修复改为必须包含实际Mux，唯一状态owner/实例/代次/ACK保持，WSL/ARM64通过，提交89aa34e。失败包中15527个状态均held，PWM/轮速全零，不能算运动测试。

客户端修复后用户执行两次relative0.5m，均SUCCEEDED且自行停稳，地面合计83cm，无hold/急停/挪动，车体朝向偏左。两次目标同步记录为straight-client-fixed.bag，正在逐goal核对定位与命令/发送/MCU停车时序。未将总距离或最终零直接判为完整实车验收。
