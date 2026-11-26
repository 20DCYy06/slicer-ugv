# B 户外 RTK 定向优化

现场 B RC1 检出和发布清单记录 05f14ef。安装内的滚动地图 YAML 与 field launch 和该源码一致，旧的编译记录仍为 80e49bf。本轮在 fix/rtk-b-field-navigation 上继承同基线的 549f958 航向锁并干净构建独立候选 20261126-rtk-b-field-nav-rc1。

## 最近三份录包

- 062608。一条 map 任务在 106.1 s 到达，原始 RTK 残差 0.092 m。其间有 103 条非空全局计划及 4 条空计划，低速角速度多次换向。另一个目标在录包结束前 2 s 发出，结果缺失。
- 063735。odom 配准段成功。之后的 map 目标在余下 88.8 s 内无终态，非空全局计划 89 条，低速角速度多次换向。
- 064123。首个 map 任务在 38.8 s 被标为振荡中止。后两条分别得到原生 Action 的 PREEMPTED，最终一条在 32.86 s 得到 SUCCEEDED。包内缺最后一条地理复核。已有 native Action 抢占证据，桌面与 GeographicClient 的普通任务准入是另外的限制。两次成功任务进入最后 1.50 m 后分别用时 19.76 s、13.96 s。

## 行为和参数

- 普通桌面 2D 目标和 WGS84 目标复用 move_base 的 Action 抢占。桌面 send 保留点击时的 ROS 时间戳，延迟发送的旧请求按 Action 时间次序处理。首次任务跨越配准时保留原始时序，允许在配准超过 30 s 后交给新的 map Action。等待旧 WGS84 退出时只保留最新目标。校准、空地专用任务、人工 Mux 接管和急停保留各自的所有权检查。
- 1.35 m 弦线起转，转向误差 0.12 rad 且停稳后采一帧新鲜原生全局占据图。在首条路径的连续已观测前缀中找边界，沿弧长回退 0.50 m。距锚点小于 0.80 m 后交接最后一个全局计划。已经处于锚点容差内的短前缀直接按正常 Guard 路径处理。
- Guard 独立检查承诺路径前方 1.20 m 上的足迹与已知致命格。持续输出零速并回报同一承诺的新鲜硬障碍信号超过 2.0 s，DWA 才可异常解锁。未知和地图边界仍由原 Guard 检查运动安全。
- 到点容差为 DWA 与地理复核 0.20 m、航向 0.30 rad。DWA min_vel_x=0.0、min_vel_trans=0.08 m/s、min_vel_theta=0.25 rad/s，终段减速度预算为 0.75 m/s²。Guard 独立制动包络继续限制实际出口。
- 对称绕行的侧别用固定任务起终点轴与路径显著前瞻偏移的叉积，横向死区 0.25 m。对向候选比已选道路的可行剩余长度缩短超过 25% 时才允许换边，或者原侧剩余路线在当前栅格已无法安全接续。有效的同侧剩余路线用当前障碍硬掩码复查。

隔离脚本为 tools/rtk/tests/check_field_navigation_isolated.sh <候选工作区>。实车未在此任务中启动。回退时退出候选桌面，再用原 RC1 的完整安装和入口。

## RC2 启动回归

现场 RC1 配准完成后，真实 map launch 的 Guard 读取到 `min_vel_x=0.04`，触发前向 DWA 必须为零的参数校验。Guard 为 required 进程，退出后 map launch 关闭 Mux，UI 留在 odom 并显示任务受阻。RC2 将这一参数恢复为 0.0，保留平移速度阈值 0.08 m/s、角速度阈值 0.25 rad/s 和终段减速度预算。

隔离脚本新增真实 map launch 启动检查，使用独立 ROS Master 和合成 TF 验证已安装的 Guard、Mux 启动保持零速、move_base Action 就绪。RC2 使用独立发布 `20261126-rtk-b-field-nav-rc2`，启动方式与 RC1 相同。现场证据和隔离结果分别保存在该发布的 `logs/rc1-map-startup-failure.log` 和 `logs/field-navigation-isolated.log`。如需回退，退出候选桌面后切回原 `20261126-rtk-b-grasp-v19-rc1`。
