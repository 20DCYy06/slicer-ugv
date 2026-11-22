# 空地任务 V2

本文定义空地任务 V2 的命令、执行语义和状态字段。UAV 的 `ag_uav_client` 发送命令和飞机状态，UGV 的 `ag_ugv_task` 保存路线并驱动已有导航。两个节点各上各自的 ROS master。协议按状态快照工作，不需要查询服务、命令前读取状态或逐条等待回执。

## 话题

| 话题 | 类型 | 流向 |
|---|---|---|
| `/air_ground/v2/uav_intent` | `std_msgs/String` JSON | UAV 本机的调用方 → UAV gateway |
| `/air_ground/v2/command` | `std_msgs/String` JSON | UAV gateway → UGV |
| `/air_ground/v2/state` | `std_msgs/String` JSON | UGV → UAV |
| `/air_ground/v2/uav_state` | `std_msgs/String` JSON | UAV → UGV |

默认 FKIE 只同步后三项。本地 intent 留在 UAV。部署和话题验收见 [实现与操作指南](implementation_guide.md)。

## 命令

所有本地 intent 都带 `op` 和由任务发起方创建的 `task_id`。gateway 加 `version: 2`、递增整数 `seq` 和 `sent_at`（ROS 时间秒），一次发送。车端当前只接受一个逻辑命令源，按 `seq` 忽略旧消息。正常运行时使用同一 gateway 序列；系统时钟回拨或重启后要避免序号倒退。命令非锁存，断线不补发，没有自动重试或重放。

| `op` | 额外字段 | 行为 |
|---|---|---|
| `SET_TARGET` | `target: {lat, lon}` | 首次建任务；同任务后续只更换终点，保留待经过 MVP 和暂停状态。 |
| `INSERT_MVP` | `mvp_id`、`point: {lat, lon}`，可选 `before_id` | 默认追加在最终目标前。有 `before_id` 时插到指定待经过 MVP 前。 |
| `DEL_MVP` | `mvp_id` | 删除还未经过的点。 |
| `UPDATE_MVP` | `mvp_id`、`point: {lat, lon}` | 改坐标，保留顺序。 |
| `PAUSE` | — | 请求停车，保留路线。暂停期间仍可编辑。 |
| `RESUME` | — | 从暂停或阻塞恢复，按剩余路线执行。 |
| `STOP` | — | 请求停车，确认停稳后结束任务并清空路线。 |

坐标是 WGS84 地面点。MVP ID 在同一任务内不重复使用；已经过或已删除的 ID 不能再修改。没有任务时先发 SET。STOP 或完成后用新的任务 ID；STOPPING 时不接受改路线或恢复。若丢了一条增量命令，以 UGV 快照为准，由上层决定下一步；第一版无路线快照替换和历史查询。

## 执行与状态

路线为 `remaining_mvps → target`。途经点进入通过半径就完成该段，默认 0.25 m。终点复用车端的导航成功、停车和地理到达复核。只在当前点的 ID 或坐标变化时中断当前段；取消未确认时不派发新 Action。导航与定位、停车、外部 hold 门禁都在 UGV 本地处理。只有 UGV 才决定是否接受并执行。

两种状态共用以下形状，发送方不同。

| 字段 | 含义 |
|---|---|
| `version`、`vehicle_id`、`updated_at` | 协议版本、`ugv` 或 `uav`、本帧时间（ROS 时间秒）。 |
| `vehicle_position` | `lat`、`lon`、`measured_at`、`valid`。无有效新鲜坐标时为 null 坐标。UGV 读车端已补偿的 RTK 观测，UAV 读配置的 NavSatFix。 |
| `task_id`、`phase`、`reason` | 任务 ID、车型自己的状态及原因。UGV 有 IDLE、READY、SWITCHING、EXECUTING、PAUSING、PAUSED、STOPPING、STOPPED、COMPLETED、BLOCKED。UAV phase 来自可选状态源，缺省为 null。 |
| `active_point` | UGV 当前实际派发的段，含 `kind: MVP/TARGET`、`mvp_id`（终点时 null）、`lat`、`lon`，无活动段为 null。切换时可暂时显示正在取消的旧点。 |
| `remaining_mvps`、`target` | 有序的 `{mvp_id,lat,lon}` 和 `{lat,lon}` 终点。UAV 不拥有车端路线，其相应字段为空。 |
| `last_command` | `task_id`、`seq`、`op`、`sent_at`、`received_at`、`accepted`、`reason`。UGV 显示最近处理的一条，UAV 只显示最后发布的一条，接受与接收时间在 UAV 上为 null。 |

状态变化时发布，运行平稳时每秒一帧。一次状态快照可能覆盖较早的命令结果。`accepted` 表示车端接受意图；执行进度和完成看 `phase`。两端时钟同步后才可比较收发时间。断网不会让车自动停车。

## 整车集成和独立 AG_Client

整车的 `ag_uav_client`、`ag_ugv_task` 和 `slicer_air_ground_msgs`
由整车仓库维护。[整车导入 `26c0a4b`](https://github.com/20DCYy06/slicer-ugv/commit/26c0a4b)
与独立 [AG_Client `a2003d4`](https://github.com/jiazhuguanhua/AG_Client/commit/a2003d4d812aadabba4dda0bf18f3bb4a7f53572)
的 35 项路径和 Git blob 一致，同批文件另有整车适配和文档重排。整车之后独立实现了首次 WGS84
目标的导航准备和自动配准。

独立 AG_Client 的车端参考检出为 `/home/ugv/ag_v2_ws/src/AG_Client`。
`dev/ground-server-new-protocol` 上核对的提交是
[`bd6f04c`](https://github.com/jiazhuguanhua/AG_Client/commit/bd6f04ca5c80bdeb0bbd1bce04a93e2778fe4107)。
该版本使用自己的工作区、安装环境和维护分支。调用方与执行端要匹配同一通信规格。

| 对照项 | 本整车 V2 | 独立 AG_Client `bd6f04c` |
| --- | --- | --- |
| `/air_ground/v2/command`、`uav_intent` | `std_msgs/String` JSON | `slicer_air_ground_msgs/V2Command`，MD5 `34cb6711ac2357e343c5c0d43ede536e` |
| `/air_ground/v2/state`、`uav_state` | `std_msgs/String` JSON | `slicer_air_ground_msgs/V2State`，MD5 `7adc9545db9549793d4cbb1fc3634b0d` |
| 收到首次合法导航目标 | 车端可等待共享准备并在配准后继续 | 执行端在校验路线时需要可用的注册状态和 `map` 位姿 |
| 二维码上报 | 文首所列的路线命令 | 支持 `REPORT_QR` 和 `CompressedImage`。最近受理的图片发布在车端本地 `/air_ground/v2/qr_capture` |

两个 V2 使用相同的话题名和版本号，交叉组合会遇到 ROS 类型与 MD5 握手错误。
同一 Catkin 工作区的三个同名包各保留一份来源，同一运行终端查看 `rospack find`
确认包来自选定安装。两份源码的 V1 `Command` 和 `GeoPoint` 仍分别生成相同的
MD5 `b574bb9bb1240c0d2ae262d6c3feb2bf` 和 `96a85301222d503f7c5cde7f75331172`，
字段注释的差异不影响 V1 消息握手。整车导航同时只由一个任务执行器控制。

独立仓库的三个包清单均声明 `Proprietary`。整车两个 AG 包延续该声明，整车消息包的清单声明
`BSD-3-Clause`。跨仓复用时由维护者确认各自声明和发布授权的范围。

### 坐标、时序和配准

两套协议的导航 `lat`、`lon` 都是 WGS84 十进制度数，代表地面目标。
整车 V2 的终点转换采用车端保存的 WGS84 原点和本地 ENU 平面，目标以
`base_link` 的水平位置为参考，车端 GNSS 观测按杆臂补偿。`sent_at` 和 `received_at` 用于报文观测，跨机器比较时需先同步
主机时钟。`seq` 是单一逻辑发送源的递增去重序号，网关续跑和重启也要保持递增。
独立版本另外给二维码采样和 JPEG 图像保留各自的测量时戳及图像头。

整车自动配准使用车端 GNSS 接收时戳，把同一个 ROS 纳秒戳的 `/gnss/fix` 与
`/gnss/fix_quality` 配对，筛选新鲜合格固定解，再用该戳在本车 `odom` 的相邻样本间插值。
它处理杆臂和 WGS84 原点，拟合完成后仍需等本车全局 EKF 和 `map` TF 更新。
GNSS 时间采用主机接收时刻的 ROS 映射，报文时戳和 UAV 位置只服务各自的通信或状态显示消费。

### 跨团队验证约定

联调前由双方记录各自仓库提交、ROS 包物理来源、生成的消息 MD5、FKIE 白名单和时钟来源。
对齐候选协议时还要确认目标与二维码的坐标语义、首任务待配准行为、暂停恢复、停止和完成证据。
先用两个隔离 ROS Master 检查端点与消息契约，再分别验证已有配准和冷启动的任务接续。

`bash tools/check_offline.sh` 运行仓库现有的纯 Python 和 Qt offscreen 测试。
它覆盖地理几何、导航准备、V1/V2 执行语义、Guard/Mux 和桌面流程。依赖
`python3-numpy`、`python3-yaml` 和 `python3-pyqt5`。含 ROS 消息的 GNSS 适配器
测试 `test_fix_quality.py` 和 `test_gnss_global.py`、桌面地图测试
`test_map_display.py`、V2 话题 `check_v2_ros.py` 和首任务接续
`check_auto_registration_ros.py` 使用独立 Noetic 环境验证。
