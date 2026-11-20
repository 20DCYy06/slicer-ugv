# 空地任务接口 V1 · 队友接入说明

协议版本：`1`。更新日期：2026-11-15。当前实现位于 `workspace2/air-ground-v1-20261115`，基线提交 `76367a7d4d939c4e404313e0125a215c2dfd718e`；实现提交 `4a53b66`（包含 `98f6005`、`36ab8c4`）。电脑临时代替飞机，使用飞机将来使用的同一 ROS 消息和任务语义。

本版只处理室外 WGS84 任务：一个最终停车点，加零个或多个有序途经点；同时只有一个任务。飞机侧提供唯一、串行的逻辑命令出口，人工操作也由飞机侧统一协调。车端没有双源优先级、控制租约或独立任务 FIFO。

**验证状态：Noetic/Catkin 隔离验证 429 项通过，0 错误、0 失败、0 跳过；安装环境检查通过。** 最终实现 `4a53b66` 已完成车端 ARM64 构建安装、隔离 ROS 回归和运行文件哈希核对；电脑经局域网读取状态并收到无效任务 PAUSE 的正式拒绝回执。真实车辆任务尚未执行，不能据此宣称实车任务通过。2026-11-11 的 WGS84 导航结果属于原导航底座，不是本空地接口的实车验收；历史 tools 全回归仍有失败，不能把本次定向验证称作全部历史回归通过。分项结果见文末。

## 1. 快速开始

现场联调直接用[空地联调操作单](field/AIR_GROUND_FIELD_RUNBOOK.md)，按室内协议检查、室外飞机静止发令的顺序执行。

以下运动相关命令由现场用户执行。先按[现行工程 README](../README.md#用户现场顺序下列driverreleaseinitializesend涉及真实硬件运动)完成传感器、底盘、odom 导航中的 RTK 初始化，再保持定位/GNSS运行，退出 odom 导航并切到 map 导航。详细现场检查沿用[户外 RTK 操作单](field/OUTDOOR_RTK_20261111.md)；该操作单中的日期、IP 和输出目录是当日记录，新一轮必须使用当前地址和新目录。

初始化会实际前进，不能用启动空地节点代替。先确认 `/geographic/status` 已配准且当前注册有效；保持原 DWA、Guard/Mux/Watchdog 链。所有终端连接同一 ROS master。

车端使用我方独立目录。本轮最终版本已安装。完成上述现场导航准备后启动任务节点；无需每次重新 build：

```bash
cd /home/potato/slicer_workspace2_codex/src/Online_Navigation_with_RTK
export ROS_MASTER_URI=http://192.168.3.68:11311 ROS_IP=192.168.3.68
./run.sh tasks
```

上面 IP 是本轮实际联调地址；网络变化时更换为当前 Jetson 地址。若同名任务服务已运行，先检查状态，不要重复启动。本轮用于局域网验证的临时软件容器及其 ROS master 已关闭；正常测试时按上述顺序自行启动即可。

`tasks` 对应 `roslaunch ugv_bringup slicer_air_ground.launch`，只添加任务层；传感器、底盘、定位和 map 导航仍使用现有入口。不要并行启动另一套导航或目标发送器。任务层独占 `/ugv/move_base` 的任务目标，使用它期间不要再运行 `geographic.py send`、`goal.py send`、旧航点接收器或巡航节点。

在已 source ROS Noetic 和本接口安装环境的电脑上查看状态：

```bash
rosrun ugv_navigation air_ground.py status
```

现场测试前，在另一个车端终端开启现有只读采集器：

```bash
cd /home/potato/slicer_workspace2_codex/src/Online_Navigation_with_RTK
EVIDENCE_DIR="evidence/air-ground-$(date +%Y%m%d-%H%M%S)"
./run.sh shell python3 tools/collect_odom_test.py --navigation --duration 900 --output "$EVIDENCE_DIR"
```

它已包含正式 command/state、Action goal/result、配准、RTK、odom、plan、costmap 和速度链；Ctrl-C 只结束记录，不停车。先用短距离单终点，再测途经点替换、暂停/恢复和取消；每组运动由现场用户确认安全后发令，记录实际停车、干预及同一车体参考点到标记的实测距离。

真实目标必须来自已保存的现场采点文件。采点仍使用既有 `geographic.py capture`；文件可从旧配准会话沿用，但不能把旧会话的局部 map 坐标当成新会话坐标。电脑端填写该真实文件的本地路径：

```bash
TARGET_FILE=/absolute/path/to/captured_target.json
REQUEST_FILE=/absolute/path/to/new_request.json
```

**先 readiness release，再提交首条 SET。** 现场用户先按既有流程核查定位、障碍、底盘、接管状态，并在车端显式执行一次 readiness release：

```bash
./run.sh shell rosrun ugv_slicer_control control.py state
./run.sh shell rosrun ugv_slicer_control control.py release
```

然后电脑端发送真实目标：

```bash
rosrun ugv_navigation air_ground.py set --target-file "$TARGET_FILE" --save-request "$REQUEST_FILE"
```

任务节点不会解除用户或其他来源留下的外部 hold。若误在 startup/external hold 时先发 SET，命令可能已受理，但执行会转为 BLOCKED；现场用户完成 readiness release 后，还须显式 `air_ground.py resume` 才继续该任务。启动 `tasks` 不启动驱动、定位、初始化或 release，也不因启动/停止一个从未使用的空闲任务节点取得控制权。需要连续观察时：

```bash
rostopic echo /air_ground/state
```

暂停、继续和取消使用同一电脑工具，逐条串行执行：

```bash
rosrun ugv_navigation air_ground.py pause
rosrun ugv_navigation air_ground.py resume
rosrun ugv_navigation air_ground.py cancel
```

`accepted=true` 只表示命令受理。最终完成须看到 `phase=COMPLETED`，并检查 `last_task_result` 中的 Action、停车和地理复核结果。结束时先按现有 `control.py hold` 现场接管，再停止任务进程和需要停止的导航进程。

## 2. 接口与安装

| 接口 | ROS 类型 | 发布方式 / 用途 |
| --- | --- | --- |
| `/air_ground/command` | `slicer_air_ground_msgs/Command` | 发送端非锁存，有限队列，单次请求只发布一次 |
| `/air_ground/state` | `std_msgs/String` | UTF-8 JSON 最新状态快照，锁存，队列 1；每 0.2 s 发布最新快照 |
| `/ugv/move_base` | `move_base_msgs/MoveBaseAction` | 车内既有导航 Action，由任务层调用 |
| `/geographic/status` | `std_msgs/String` | 既有地理配准/定位状态，任务层复用 |

命令和状态就是飞机的正式接口，不另设测试 topic，不要求飞机回 ACK。任务层经原导航、Guard、Mux、Watchdog 到底盘，不直接发布速度或 PWM。任何后续速度接口只能进入 Guard 的 raw 入口，禁止旁路到真实底盘 `/cmd_vel`。

最小消息包只有 [slicer_air_ground_msgs](../src/slicer_air_ground_msgs/package.xml) 及两个 `.msg` 文件，不依赖导航算法、驱动或车体工程。队友已有 ROS1 Noetic/Catkin 时，可把此包放进自己的工作区 `src` 后构建：

```bash
cd /path/to/air_ground_ws
catkin_make --only-pkg-with-deps slicer_air_ground_msgs
source devel/setup.bash
rosmsg show slicer_air_ground_msgs/Command
```

双方须使用完全相同的 `.msg` 定义；不要只手写同名 Python 类。Python 发送端还需要 Noetic 自带的 `rospy`、`std_msgs`。完整本工程构建安装后可用上述 `rosrun ugv_navigation` 命令；只分发消息包时，可同时分发 [air_ground.py](../src/ugv_navigation/scripts/air_ground.py) 与同目录的 [air_ground_wire.py](../src/ugv_navigation/scripts/air_ground_wire.py)，然后直接运行 `python3 /path/to/air_ground.py status` 等同参数命令。

队友可直接使用随本轮交付的 `slicer-air-ground-v1-sdk.zip`：仅包含消息包、正式 CLI、wire helper、rospy 示例和本说明，不带导航/驱动依赖或凭据。独立 Noetic `--network none` 构建安装已验证。完整主工程才提供 `rosrun ugv_navigation`；SDK 使用 README 中的 `python3 scripts/air_ground.py` 入口。

本版生成的 `slicer_air_ground_msgs/Command` MD5 为 `b574bb9bb1240c0d2ae262d6c3feb2bf`，可用 `rosmsg md5 slicer_air_ground_msgs/Command` 核对。

### 同一 master 与电台局域网

优先让电脑/飞机直接连接现有车端 ROS master；电台只提供局域网，无需额外同步层。项目运行入口通过 host 网络容器连接现有 master；实际默认来自 [slicer_runtime.env](../src/ugv_bringup/config/slicer_runtime.env)，本地默认是 `127.0.0.1`，远端电脑不能直接照用回环地址。

现场核实当前 Jetson 和电脑地址后，在所有对应终端设置；下面大写地址词必须替换成真实当前 IP：

```bash
# 车端各终端
export ROS_MASTER_URI=http://JETSON_IP:11311
export ROS_IP=JETSON_IP
unset ROS_HOSTNAME

# 电脑 / 飞机端各终端
export ROS_MASTER_URI=http://JETSON_IP:11311
export ROS_IP=COMPUTER_IP
unset ROS_HOSTNAME
```

也可统一维护车端 `slicer_runtime.env` 的相应值。ROS1 需要 master 和节点地址双向可达，不能仅证明 11311 可连；双方时钟也须同步，因为工具检查状态 ROS 时间戳且拒绝未来时间。不要沿用旧操作单的静态 IP，不修改原 GNSS/NTRIP 链路来解决此接口的连通问题。

只读检查：

```bash
rostopic type /air_ground/command
rostopic type /air_ground/state
rostopic info /air_ground/command
rosrun ugv_navigation air_ground.py status
```

应分别显示正式 Command、`std_msgs/String`，且任务命令只有一个逻辑发送出口。节点掉线/重连后，旧命令不会由 topic 自动重放。

若现场继续使用已有 FKIE 双 master 通信，可沿用[独立通信工程](../../conmunication/README.md)，两端白名单直接同步 `/air_ground/command` 和 `/air_ground/state`，保持原名、原消息类型及发布端锁存属性。不要同时再增加另一条命令转发路径。两端的正式消息发布进程、任务节点及通信容器环境都须能解析同一版 `slicer_air_ground_msgs`。

通信 `run.sh` 在同级 RTK 工程已有 `install/setup.bash` 时，只读挂载该 install 到 `/rtk_install`，供新建的 sync/discovery/relay 容器在 source Noetic 后 `source /rtk_install/setup.bash --extend`；不会自动构建或使用旧工程。没有该 install 时保留原标准状态通信能力，但正式任务接入前必须先构建消息包并创建带安装挂载的通信实例。已存在的容器不会因 `start` 或 `reload-sync` 获得新挂载，需按当前实例清单明确停止/重建自己的新前缀通信容器；旧 `ugv-*` 容器及其旧工程挂载保持原样，不能作为本版已切换的证据。飞机侧由队友同样构建并 source 消息包。

## 3. 完整消息定义

定义文件：[GeoPoint.msg](../src/slicer_air_ground_msgs/msg/GeoPoint.msg)、[Command.msg](../src/slicer_air_ground_msgs/msg/Command.msg)。

### GeoPoint

| 字段 | ROS 类型 | 必填 / 缺省 | 语义 |
| --- | --- | --- | --- |
| `latitude` | `float64` | 点有效时必填 | WGS84 纬度，度，有限数，`[-90,90]` |
| `longitude` | `float64` | 点有效时必填 | WGS84 经度，度，有限数，`[-180,180]` |
| `has_altitude` | `bool` | 缺省 `false` | 是否明确提供椭球高 |
| `altitude_ellipsoid_m` | `float64` | 仅 `has_altitude=true` 时使用 | WGS84 椭球高，米，有限数；不是 MSL / 海拔 |
| `has_yaw` | `bool` | 缺省 `false` | 是否明确提供最终朝向；途经点必须为 `false` |
| `yaw` | `float64` | 仅 `has_yaw=true` 时使用 | 弧度，WGS84 ENU 正东为零、逆时针为正；只允许最终点 |

点表示希望车辆 **base_link 原点的水平地面投影**到达的位置，不能直接填写飞机飞行位置。已采集目标已做一次水平天线杆臂补偿，读取时不再次补偿。车端复用当前 WGS84/ENU 配准；相同的 frame 名 `map` 不代表两个独立系统已有共同原点和轴向。

未提供高度时，以当前任务原点的椭球高进行二维转换，相当于该原点 ENU 的 Up=0 约定；这不是测得的地面高，也不把缺失值解释为真实椭球高 0 m。已有采点文件保留其原有高度约定：水平参考为 base_link，但 Up 可能仍保留 GNSS 天线值，不能冒充实测 base_link 三维高。飞机飞行高度不应作为地面目标高度传入。

最终朝向未提供时，车端按当前车辆到终点的接近方位生成；若目标与车辆重合则保留当前 yaw。不会默认为世界方向零度。途经点只有位置通过语义，车端在通过范围内主动推进，不要求每个途经点最终姿态对准。

JSON 状态及保存请求中的点使用 `latitude`、`longitude`、`altitude_ellipsoid_m`、`yaw`，后两个未指定时为 `null`。正式 ROS 消息用 `has_*` 表示同一含义；标志为 false 时对应数值不参与命令语义，发送时仍建议保持默认 0。

### Command

| 字段 | ROS 类型 | 规则 |
| --- | --- | --- |
| `protocol_version` | `uint8` | 必填，固定为 `1` |
| `server_id` | `string` | 必填，必须等于最新状态中的本次接收端启动身份 |
| `sequence` | `uint64` | 正整数；本次接收端启动内全局递增，可跳号，不等待补齐 |
| `request_id` | `string` | 必填非空；标识一次请求；重发必须保留完全相同内容 |
| `task_id` | `string` | 必填非空；SET 使用新身份，其他操作指向当前未结束任务 |
| `operation` | `string` | 五个区分大小写的大写操作名之一 |
| `final_goal` | `GeoPoint` | SET 必填；其他操作须语义为空，即纬经度为 0、可选标志为 false，推荐保持完整默认消息 |
| `via_points` | `GeoPoint[]` | 有序列表；只允许 SET / REPLACE，其他操作为空 |

消息中提供 `SET_TASK`、`REPLACE_VIA_POINTS`、`PAUSE`、`RESUME`、`CANCEL` 字符串常量。`server_id` 是随机的接收端启动身份，用来挡住重启前的旧消息；它不是控制租约、来源授权或操作优先级。

## 4. 五种任务操作

| `operation` | 电脑命令 | 精确定义 |
| --- | --- | --- |
| `SET_TASK` | `set` | 完整提交终点及可选途经点。替换旧任务、旧终点与全部旧未完成途经点；旧任务结果记为 `REPLACED`。须使用新的 `task_id` |
| `REPLACE_VIA_POINTS` | `replace-via` | 整体替换当前未结束任务的全部剩余途经点，包括正在前往的途经点；保留终点 |
| `PAUSE` | `pause` | 取消当前导航并按既有机制 hold/停车；保留任务、终点及剩余路线 |
| `RESUME` | `resume` | 在定位/控制条件满足时，从当前位置继续剩余路线；不回放旧轨迹 |
| `CANCEL` | `cancel` | 取消导航并停车，结束任务并清空剩余途经点；之后 RESUME 不能复活该任务 |

路线 A→B→G 中替换途经点为 C、D，剩余路线变为 C→D→G。空列表变为直接前往 G，不表示暂停或取消。SET 新终点 H 则旧 G 和旧途经点一起失效，执行新消息包含的路线。

整条命令与整批点先校验，通过后一次性替换；无效批次不清空或破坏原任务。车端接收边界只解码字段，保留越界值、NaN、错误版本和不应出现的负载，由同一任务状态机做身份去重及语义拒绝。被拒绝的请求身份也会记录，不能把同一 request_id 的无效坐标改成有效坐标后重新提交；修正必须创建新请求。暂停期间 SET 或 REPLACE 可更新意图，但仍保持暂停，直到明确 RESUME。已完成或已取消任务不接受迟到路线更新。

当前实现每次换段/换路线先取消旧 Action，确认旧执行终止后才发送新目标；发出 cancel 不等于旧执行结束。回调通过任务/路线代次和执行 token 归属检查，旧成功、失败或取消结果不能推进新路线。途经点在配置通过半径内触发换段，赶在 DWA 对该点做最终姿态对准之前取消；不运行完整终点停车/地理复核。换段允许减速或短暂停顿。

必经点失败或无法到达时停止推进并保留任务及剩余路线，状态给出原因，不默认跳点。可根据现场原因显式更新路线或恢复。最终目标必须依次通过 Action 成功、停车窗口和地理复核，才标记任务完成。

### Hold 与恢复

现有外部 Mux hold 始终有效。PAUSE/CANCEL 使用既有 hold 机制；明确 RESUME 后，仅能解除本任务节点自己产生、且 Mux instance / takeover generation 与取得 hold 时完全相同的暂停 hold。任务层把预期二元组传入现有 `control.py release`，CLI 在读取新鲜状态后再次核对，防止检查后用户接管的竞态；普通用户 control.py 的默认行为和 Mux/Guard 策略保持不变。

用户再次接管、Mux 重启或代次变化后，节点不能把新的外部 hold 认领成自己的，须现场用户重新检查并按现有控制流程处理。SET、REPLACE、收到新坐标和恢复通信都不会解除外部保护停止。终点完成后节点也会持有自己的停车 hold；后续明确创建新任务时，可在同一归属检查通过后进入新任务，无需人为反复 release 未变化的自有停车 hold。

## 5. 电脑命令与飞机示例

所有命令串行发送；不要在两个终端同时发任务命令。工具先获得新鲜状态，再自动生成 `sequence=last_sequence+1`、UUID `request_id`；SET 还生成新 UUID `task_id`，其他操作使用状态中的当前任务编号。发送端重启不需要恢复本地计数器或申请租约。

一次提交终点和多个途经点，文件顺序就是经过顺序：

```bash
rosrun ugv_navigation air_ground.py set --target-file "$TARGET_FILE" \
  --via-file /absolute/path/to/first_captured_via.json \
  --via-file /absolute/path/to/second_captured_via.json \
  --save-request /absolute/path/to/new_set_request.json

rosrun ugv_navigation air_ground.py replace-via \
  --via-file /absolute/path/to/new_captured_via.json

# 空列表：清除全部剩余途经点，保留当前终点
rosrun ugv_navigation air_ground.py replace-via
```

目标文件必须是既有 `geographic.py capture` 输出的 JSON：`kind=slicer_geographic_target`、整数 `version=1`、`datum=WGS84`、`physical_reference=base_link_horizontal`，并包含有限的纬度、经度、椭球高。`capture` 的旧原点和代次是来源记录，不绑定新任务配准。文件默认不指定 yaw；最终点可增加 `--yaw`，途经点不支持 yaw。

也支持显式真实 WGS84 输入：`set --latitude LAT --longitude LON [--altitude-ellipsoid HEIGHT] [--yaw RAD]`。必须把 LAT/LON 换成合法、实测或正确变换得到的车辆目标，不能照抄虚构坐标。此模式不能混用 `--target-file`。

各子命令的通用等待参数及默认值：

| 参数 | 默认值 | 作用 |
| --- | --- | --- |
| `--state-timeout` | 5 s | 等待新鲜状态的上限 |
| `--state-max-age` | 2 s | 状态本地接收年龄及 `updated_stamp` 年龄上限；未来时间拒绝 |
| `--connect-timeout` | 3 s | 发送前等待命令订阅者；只解决传输连接 |
| `--receipt-timeout` | 3 s | 单次发布后等待本请求受理结果；不等待任务完成 |

选项放在子命令之后，例如 `air_ground.py status --state-timeout 8`。命令退出码：`0` 表示明确收到 `accepted=true`，`1` 表示明确拒绝，`2` 表示本地验证、文件、状态、传输或回执异常；`status` 的 `0` 表示取得新鲜状态。退出 `2` 或回执超时可能发生在车端已受理之后，不能直接认为未执行。

### 保存与重发同一请求

五种新命令均支持 `--save-request PATH`，在发布前以新文件保存准确请求，拒绝覆盖已有文件。需要确认同一请求时：

```bash
rosrun ugv_navigation air_ground.py status
rosrun ugv_navigation air_ground.py replay "$REQUEST_FILE"
```

`replay` 保留原 `server_id`、`sequence`、`request_id`、`task_id` 和全部负载，不分配新身份。不要编辑文件后再当作同一请求重发。普通命令每次执行都会创建新请求，因此超时后直接再次运行 `set` 会创建新的替换任务。

工具只发布一次，不自动重试、不自动过期任务、不因等不到回执取消任务。无命令订阅者时在连接上限内退出，并明确表示未发布；已发布但回执丢失时结果未知，先查状态，必要时重放准确保存的请求。

### 最小 rospy 发布 / 订阅示例

可运行的 [air_ground_rospy.py](../src/ugv_navigation/examples/air_ground_rospy.py) 从 argv 读取真实采点文件，获取当前启动身份/序号，用正式消息非锁存发布一次 SET，然后订阅异步状态：

```bash
rosrun ugv_navigation air_ground_rospy.py "$TARGET_FILE"
```

只有消息包及示例源文件时，保留示例与 scripts 的相对目录布局，或把 `air_ground_wire.py` 放入 Python 搜索路径后运行：

```bash
python3 /path/to/ugv_navigation/examples/air_ground_rospy.py "$TARGET_FILE"
```

它使用 `rospy.Publisher('/air_ground/command', Command, queue_size=1, latch=False)` 和 `rospy.Subscriber('/air_ground/state', String, callback, queue_size=1)`。消息发布后没有等待飞机 ACK 的步骤，状态打印也不驱动车端任务推进。飞机自行实现时只需同样的正式消息与 topic；可以复用 wire helper，也可按字段表直接构造消息。

## 6. 状态与完成判据

`/air_ground/state` 是单个最新 JSON 快照，不是可靠事件队列。状态锁存，`queue_size=1`，每 0.2 s 发布最新快照；执行线程通常每 0.05 s 轮询并更新缓存。状态变化在后续发布周期可见，有限发布队列不等待飞机 ACK，不因无订阅者、慢订阅者或飞机掉线阻塞导航。

重连后可直接看到最新快照，但须同时理解两个时间戳：`updated_stamp` 是本次反馈发布的 ROS epoch 秒，`execution_updated_stamp` 是工作线程最后刷新执行快照的 ROS epoch 秒。发布新鲜不代表工作线程刚完成执行推进；观察端可据此辨认执行快照停滞。CLI 按 `updated_stamp` 及本地接收时间检查反馈新鲜度，不能把收到锁存消息本身当作新鲜。

| 字段 | 类型 | 含义 |
| --- | --- | --- |
| `protocol_version` | integer | 固定 `1` |
| `server_id` | string | 本次接收端启动身份 |
| `last_sequence` | integer / null | 本启动已处理的全局序号水位；初始为 null |
| `updated_stamp` | number | 本次状态发布的车端 ROS epoch 秒；供反馈新鲜度检查 |
| `execution_updated_stamp` | number | 工作线程最近刷新执行快照的 ROS epoch 秒 |
| `task_id` | string / null | 当前或最近任务身份 |
| `revision` | integer | 意图/路线代次，变化使旧执行 token 失效 |
| `paused` | boolean | 是否暂停或因阻塞保留路线 |
| `phase` | string | 任务意图阶段，见下表 |
| `runtime_phase` | string | 执行子阶段：`NAVIGATING`、`SWITCHING`、`VERIFYING`，或同名任务阶段 |
| `reason` | string | 暂停、阻塞、失败等原因；空字符串表示无附加原因 |
| `final_goal` | object / null | 原始 WGS84 最终点；可选高度/yaw 用 null |
| `remaining_via_points` | array | 全部未完成途经点，包含当前正在前往的途经点 |
| `active` | object / null | 当前段的 `token`、`kind`（via/final）、`point`、`task_id`、`revision` |
| `current_navigation_target` | object / null | 最近生成的实际导航目标；含 `frame`、`map_xyz`、`yaw`、`generation` 及下文归属字段 |
| `action` | object / null | 最近 Action 观测：`state`（actionlib 数字状态）、`text`、`observed_stamp` |
| `arrival` | object / null | 最近终点复核：`target`、`generation`、`evaluated_stamp`、`move_base`、`stop`、`geographic` |
| `last_error` | string | 执行/停车或 hold 未确认等补充错误；与 `reason` 一起查看 |
| `stop` | object / null | 最近暂停、取消或启动清理的停车窗口证据，与终点 arrival.stop 分开 |
| `startup_checked` | boolean | 是否已读到首次新鲜 Action 状态并检查遗留执行 |
| `startup_recovery` | object / null | 发现遗留 Action 的原因/时间、stop/stop_confirmed；确认后保留该次证据 |
| `cancellation_pending` | boolean | 是否仍在等已拥有 Action 的取消终止确认 |
| `last_command` | object / null | 最近命令身份、操作、序号、`accepted`、`reason`、`duplicate` |
| `last_task_result` | object / null | 最近任务的 `task_id`、`status`、`reason`、`result`，最终检查结果在 result 中 |
| `via_pass_radius_m` | number | 当前实际途经点通过半径 |
| `navigation_action` | string | 当前实际导航 Action 名，默认 `/ugv/move_base` |
| `physical_reference` / `datum` | string | 固定 `base_link_horizontal` / `WGS84` |
| `transport_error` | object / null | latched 命令拒绝或入口队列满等：`reason`、`request_id`、`stamp` |
| `software.files_sha256` | object | 正在运行的 6 个任务/地理脚本及 helper 的文件 SHA256，供安装版本核对 |

`current_navigation_target` 的归属字段为 `task_id`、`route_revision`、`segment_token`、`segment_kind`（`via` / `final`），配准身份为 `generation`。同时保留原输入 `requested_wgs84`、`height_policy`、`yaw_policy`、`task_origin` 和转换时刻 `stamp`。目标在段结束后可继续保留为最近一次目标；必须结合当前 `active` / 阶段及上述归属判断，不能只凭对象非空就认定仍在执行。

`last_command` 是本次实际返回的回执；重放旧请求时也会公开它原来的受理结果并标记 `duplicate=true`，不重新执行。状态不是所有历史回执的存储接口。入口拒绝锁存命令，命令队列最多 32 条，每轮最多处理 8 条，留出导航/停车轮询时间；队列满等传输问题看 `transport_error`，不应误解为任务已受理。

| `phase` | 含义 |
| --- | --- |
| `IDLE` | 接收端本次启动没有任务 |
| `READY` | 意图已受理，等待运行条件或换段完成 |
| `EXECUTING` | 当前任务段执行中；结合 `runtime_phase` 判断导航/复核 |
| `PAUSING` | 正在取消旧执行并确认停车 |
| `PAUSED` | 暂停完成，保留路线 |
| `BLOCKED` | 导航/定位/取消/到达检查等阻塞，保留路线及原因，等待显式处置 |
| `CANCELING` | 正在取消，尚不能把请求本身当作停止确认 |
| `CANCELED` | 已取消并结束，路线已清空 |
| `COMPLETED` | 终点 Action、停车与地理复核全部通过 |

不要仅凭 Action 的 SUCCEEDED 宣告物理到达。终点沿用 `arrival_review`，分别记录 `arrival.move_base.state=3`、`arrival.stop.confirmed=true` 和 `arrival.geographic.confirmed=true`；完成结果同样保留在 `last_task_result.result`。停车要求连续新鲜 `/odom` 安静窗口，地理复核使用该停车窗口内原始合格 RTK 观测经一次杆臂补偿后的水平残差，并公开 `qualified`、`horizontal_residual_m`、`tolerance_m`、`stopped_window` 和 `reason`。它们全部成立才完成。原始 RTK 同时参与融合定位，因此该地理复核不是独立测量真值；厘米级绝对精度仍须现场独立测量。

常见拒绝/阻塞原因与处理：

| 现象 / 原因 | 含义与处理 |
| --- | --- |
| `unsupported protocol_version` | 双方协议不一致，核对消息包与版本 |
| `server_id does not match this receiver boot` | 接收端已重启；读取新状态，重新明确创建任务 |
| `sequence is not newer than last_sequence` | 旧序号或多发送者冲突；先读状态，保持唯一串行出口 |
| `request_id was already used with different content` | 同请求身份被改内容；恢复原请求或用新身份发新命令 |
| `task_id does not name an unfinished task` | 指向旧任务、已完成/取消任务或不存在的任务 |
| 坐标、height、via yaw 无效 | 整批拒绝，原路线保留；修正后作为新请求提交 |
| 配准失效、时间戳过旧、注册代次变化 | 不把旧 map 坐标套入新配准；保持停车，恢复定位后显式处置 |
| 旧 Action 取消超时 / 停车未确认 | 不允许并行发新导航执行；检查原导航和控制状态 |
| 必经点失败 / 终点复核失败 | 不跳过或伪报完成；检查原因，明确改道、恢复或取消 |
| state 时间在未来 | 核对双方 ROS 时间与系统同步，不能以放大容差掩盖错误 |

## 7. 顺序、断线、重启

序号在一个接收端 `server_id` 内全局有效，不是每任务独立计数；允许跳号。通过信封检查且比水位新的命令会消耗该序号，即使后续因坐标批次、转换、任务编号或任务状态被拒绝；下一命令须重新读取最新状态。坏信封、旧序号和请求身份内容冲突不推进水位。属于本次 boot 且 request_id 非空可识别的坏信封仍缓存拒绝回执，不能改正协议版本/操作等后复用同一 request_id；错误 server_id 不跨启动缓存或取得任务控制。

同一 `request_id`、完全相同语义内容的重发返回原处理结果，并标记重复，不会重复启动、重复插入途经点或回到旧路线。同一 ID 不同内容明确拒绝，包括首次因协议、操作或坐标等无效而被拒绝后修改内容；只有属于当前接收端且请求身份可识别时才能缓存该回执。`has_* = false` 的可选数值被忽略，不会因无效数值先被边界丢弃而绕过身份检查。接收端在本次启动内保留每个可识别请求的指纹及原回执，不淘汰已处理身份；状态输出仍仅发送最新快照。回执内存随该次启动期间的唯一请求数线性增长，重启后清空并更换 server_id。不能用旧保存文件长期充当新任务模板。

任务完成或取消后，旧路线更新、旧 Action 回调和 RESUME 都不能复活任务；新任务必须显式 SET 新 `task_id`。SET 替换旧任务时，旧任务结果标明 `REPLACED`。

通信中断不新增运动租约：原有导航、定位与控制条件仍满足时，车端继续最后已受理任务。反馈无人订阅或回执丢失都不会阻止任务执行。重连订阅状态拿到最新快照，不补发完整历史；客户端丢失某条命令时，车端不会自行猜测或补齐。

任务节点重启后生成新 `server_id`，不恢复旧任务。首次收到新鲜 Action 状态时若发现遗留活动目标，先使用既有 hold/cancel-all 清理，并等待 Action 终止和原静止窗口；清理未确认时不派发新目标。`startup_recovery.stop_confirmed`、`startup_recovery.stop` 公开清理证据。没有导航状态时 `runtime_phase=WAITING_NAVIGATION`，不把逻辑 IDLE 当作物理停车确认；命令非锁存，不会在重连后自动恢复。旧 boot 的保存请求在工具端就被拒绝，车端也拒绝旧身份。发送端重启则读取当前新鲜状态，继续分配全局序号；没有额外 sender lease、持久计数器或多来源仲裁。

## 8. 参数、停止与回退

任务参数集中在 [air_ground.yaml](../src/ugv_bringup/config/air_ground.yaml)：

| 参数 | 默认值 | 作用 |
| --- | --- | --- |
| `via_pass_radius_m` | 0.25 m | 途经点水平位置通过半径 |
| `segment_timeout_s` | 120 s | 单段执行时间上限 |
| `cancel_timeout_s` | 3 s | 旧 Action 取消确认上限 |
| `action` | `/ugv/move_base` | 复用的既有导航 Action 名 |

原终点地理容差沿用 0.35 m；原停车窗口沿用 1 s、线速度绝对值不超过 0.02 m/s、角速度不超过 0.05 rad/s，并检查连续新鲜 odom 和窗口内位移。原地理状态新鲜度 1 s、RTK 观测最大年龄 2.5 s 等保持原规则。执行器停车/复核等待上限为 `max(4, stop_window_s+4)+max_fix_age_s`，以上默认值下为 7.5 s。本轮没有为了通过测试放宽 DWA、定位或终点容差；ROS 夹具缩短的测试窗口不作为实车参数。

修改本接口后可在车端用 `./tools/noetic.sh bash -c 'set -e; catkin_make -j2; catkin_make install -j2'` 只构建安装，再重启任务节点；这不会运行会重新生成底盘/感知参数的 prepare。原 `./run.sh build` 包含 prepare，只有明确需要重新生成基础配置时才使用。只修改 source 不会自动更新已经 source 的 install。部署仅面向独立新工程 `/home/potato/slicer_workspace2_codex/src/Online_Navigation_with_RTK`；队友依赖树和旧工程只读。部署前后需记录源码/配置及安装版本，不能覆盖现场未提交工作。

正常停止：现场用户先 `air_ground.py cancel` 并观察结束，必要时直接按原控制入口 hold；确认车端保持停车，再 Ctrl-C `./run.sh tasks`。紧急处置沿用既有物理急停/接管。进程退出或停止反馈订阅不能当作已停车证据。

回退时先按上述流程停车并停止任务节点，保留本轮源码/配置和现场证据，再恢复部署前记录的独立新工程版本并重建安装。退出空地任务层后，可由用户按原 README 使用既有 `geographic.py` 单目标流程；不要让两种目标发送入口并行争用 Action。回退不涉及底盘固件、原控制参数或旧项目覆盖。

## 9. 本次验证与未执行项

| 范围 | 当前结果 | 证据 / 限制 |
| --- | --- | --- |
| 电脑 CLI / wire / 目标文件 | 25 项离线单元测试通过 | `python3 -m unittest discover -s src/ugv_navigation/test -p test_air_ground_cli.py -v`；含无效请求解码、拒绝去重与同 ID 改内容回归；无 ROS master、网络、设备 |
| Python 语法 | CLI、wire、rospy 示例编译通过 | `python3 -m py_compile`；不证明 ROS 运行行为 |
| 原生纯逻辑定向测试 | 89 项通过 | core 43 + CLI 25 + executor 13 + stop boundaries 8；覆盖替换、暂停更新、取消、拒绝回执、取消屏障、旧回调、外部 hold 等 |
| 控制 hold 归属 / 采集器 / 通信配置 | 分别 3 / 18 / 5 项通过 | 预期 hold 二元组、正式任务证据采集、通信配置与只读消息安装挂载；不连接车辆 |
| Noetic/Catkin 构建、隔离 ROS 验证 | 429 项，0 错误、0 失败、0 跳过 | 包含既有 GNSS/Guard/Mux 及新正式消息 ROS 夹具；无设备、`--network none` 容器独立 master |
| 安装环境 | 通过 | `tools/check_air_ground_install.py` 核对已安装脚本/helper、生成 Command 序列化往返及官方 launch 仅一个任务节点 |
| 历史 tools 全回归 | 未全绿：196 项中 5 失败、6 错误 | 原 `tools/test_noetic.sh` 的旧 NTRIP/footprint/site/source-vs-install 预期及缺少冻结 `.generated/sensor_launch.yaml` 等基线问题；不宣称全部历史测试通过 |
| ARM64 构建与部署 | 最终 `4a53b66` 通过 | 实际 ARM64 Catkin build/install、核心用例及正式 ROS 假 Action/进程重启回归通过；安装消息 MD5 相同，运行的 6 个文件 SHA256 与本地一致 |
| 电脑→局域网→Jetson 正式接口 | 状态与拒绝回执通过 | 电脑 `192.168.3.208` ↔ Jetson `192.168.3.68`，正式 CLI replay 不存在任务的 PAUSE 得到明确拒绝；没有导航目标。未验证电台链路 |
| 真实车辆任务 | 未执行 | 至少单终点、途中换路线、暂停/恢复、取消，运动由现场用户执行 |
| 真正飞机端 / 全部电台工况 | 未验证 | 电脑代飞机通过也不等于无人机端或全部射频工况验收 |

上述原生纯逻辑测试与 Catkin 汇总有重复收录，不相加冒充总测试数。ROS 夹具通过正式 Command/topic 驱动真实任务节点，使用假 Action/RTK/TF 和真实软件 Mux，覆盖单终点复核、多途经点位置通过、运行中替换/清空路线、换终点、旧结果、暂停更新/恢复、重复/旧序号/无效批次、无反馈订阅者、重连最新状态、拒绝锁存指令，以及强制结束任务进程后新实例保持/取消遗留 Action、静止确认、旧 boot 拒绝；它没有证明真实轮地运动或电台性能。

新一轮现场记录应把任务命令、`/air_ground/state`、实际 Action 目标/结果、配准状态和停车/地理复核证据关联到 `task_id` / `request_id`。只保存本轮明确采集的新目录，不扫描或覆盖历史日志、bag 或凭据。运行前替换新输出路径，不复用旧加测目录。

室内坐标适配、独立任务队列、屏幕 UI、多来源仲裁、通信失联租约均不属于本版；后续扩展继续使用公开 ROS 接口，并保持现有导航/控制边界。

结构化结果见 [air_ground_v1_20261115.json](validation/air_ground_v1_20261115.json)。本轮软件证据：项目 `.test-results/air-ground-verified-serial.log`（429 项及安装检查）；`.test-results/air-ground-deployed-final-status.json`、`air-ground-deployed-final-negative.log`（最终版本真实局域网）；首次历史工具结果为 `.test-results/run-pEvMGiPI/output.log`。并行 ROS 测试曾因状态时间过旧触发保护停止，改为串行复核通过，未放宽阈值。车端新增证据仅位于 `/home/potato/slicer_workspace2_codex/evidence/air-ground-v1-20261115`。
