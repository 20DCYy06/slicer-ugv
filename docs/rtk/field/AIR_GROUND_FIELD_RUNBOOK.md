# 空地联调操作单

先在室内验命令和回执，再到户外让飞机静止发令、小车走一趟。在车端使用已安装的 V1。消息字段和完整示例见[接口文档](../air_ground_interface_v1.md)。

## 准备

从车端独立检出根目录运行。先按[整车操作说明](../../OPERATIONS.md)完成原生构建和设备配置，设置 `NATIVE_WS` 为该检出的 Catkin 构建目录，`JETSON_IP` 为现场车端地址。每个车端终端执行：

```bash
source "$NATIVE_WS/install/setup.bash"
export ROS_MASTER_URI="http://${JETSON_IP}:11311" ROS_IP="$JETSON_IP"
unset ROS_HOSTNAME
```

飞机与 Jetson 连同一局域网，双方 IP 可互访、时钟同步。飞机端 `ROS_MASTER_URI` 指向小车，`ROS_IP` 填飞机自己的 IP，不要填 Jetson 的 IP。飞机程序用非锁存的 `/air_ground/command`（`slicer_air_ground_msgs/Command`）发令，订阅 `/air_ground/state`（`std_msgs/String`，内容为 JSON）。同一时间只开一个任务发送端。

下面用 `ag` 简写正式客户端。在小车宿主操作终端定义：

```bash
ag() { rosrun ugv_navigation air_ground.py "$@"; }
```

飞机端若用随包 SDK，在 SDK 根目录构建一次 `slicer_air_ground_msgs`，每个终端 source 环境后定义同名函数。飞机已有正式客户端时可直接按相同步骤发令。

```bash
source /opt/ros/noetic/setup.bash
catkin_make --only-pkg-with-deps slicer_air_ground_msgs
source devel/setup.bash
rosmsg md5 slicer_air_ground_msgs/Command
ag() { python3 scripts/air_ground.py "$@"; }
```

Command 的 MD5 应为 `b574bb9bb1240c0d2ae262d6c3feb2bf`。

## 1. 室内：协议测试

小车保持静止、Motor OFF。只在小车的**任务终端**启动：

```bash
roslaunch ugv_bringup slicer_air_ground.launch
```

在**飞机端**执行 `ag status`，应看到 `phase=IDLE`、`task_id=null`。此时没有导航，`runtime_phase=WAITING_NAVIGATION` 正常，不需要 RTK。

发送一条指向不存在任务的 `PAUSE`，再原样重放。以下在飞机端执行才验证空地双向通信；在小车端执行只验证本机。飞机用自有代码时，按同一字段构造消息即可。

```bash
mkdir -p evidence
PROBE=$(mktemp -d evidence/air-ground-probe.XXXXXX)
ag status > "$PROBE/state.json" && python3 - "$PROBE" <<'PY'
import json
import sys
import uuid
from pathlib import Path

directory = Path(sys.argv[1])
state = json.loads((directory / "state.json").read_text())
assert state["phase"] == "IDLE" and state["task_id"] is None
request = dict(protocol_version=1, server_id=state["server_id"],
               sequence=(state["last_sequence"] or 0) + 1,
               request_id=uuid.uuid4().hex, task_id="indoor-probe",
               operation="PAUSE", final_goal=None, via_points=[])
(directory / "pause.json").write_text(json.dumps(request))
PY
ag replay "$PROBE/pause.json"
ag replay "$PROBE/pause.json"
ag status
```

**通过标准：** 首次回执 `accepted=false`，原因为 `task_id does not name an unfinished task`（客户端退出码 `1` 正常）；重放后状态的 `last_command.duplicate=true`，且任务仍为 `IDLE`。若状态还没刷新，再读一次。结束后在小车任务终端按 Ctrl-C，室外按下节启动。

## 2. 室外：飞机静止，小车执行

**启动。** 退出旧导航、WASD 和手柄程序。小车固定好 GNSS 天线，确认前方至少 3 m 空旷、能随车观察和急停。分别在小车的定位、底盘、导航三个宿主终端执行下面对应的命令；定位先启动，底盘先保持 Motor OFF。

```bash
# 定位终端
roslaunch ugv_bringup slicer_field_sensors.launch config_file:="$HOME/.config/slicer/gnss.yaml"
# 底盘终端
roslaunch ugv_bringup slicer_body_driver.launch
# 导航终端
roslaunch ugv_bringup slicer_navigation_runtime.launch
```

在外部 `gnss.yaml` 中设置本轮的串口、差分服务和凭据路径。操作终端先检查底盘已连接、Mux 仍保持停车，GNSS 显示 `gga_quality=4` 和 `fresh_rtk_usable=true`：

```bash
rosrun cup_gnss gnss_status.py --once --json
rosrun ugv_slicer_control control.py state
```

**自动准备。** 在现场负责人确认运行条件后发送首个合法 WGS84 目标。共享导航根据现有 RTK 状态执行有界 odom 准备，停稳并完成全局定位后自动接续原目标。用 `rostopic echo /geographic/navigation_runtime` 观察进度和阻塞原因。原直行初始化保留在导航诊断。

```bash
RUN="$HOME/workspace/slicer/data/air-ground-$(date +%Y%m%d-%H%M%S)"
mkdir -p "$RUN"
```

**已有目标时直接复用。** `geographic.py capture` 保存的是 WGS84 坐标，新配准会按当前 `map` 原点重新换算。可以先把车搬到测试起点，放稳后启动定位、完成配准，再直接预览并发送旧 `mark.json`。目标和路线要在当前地图范围内，全局代价地图在新原点的东西、南北方向各覆盖约 ±20 m。

**需要新目标时提前采点。** 在一个有效配准会话内，把车沿地面推到各个标记，保持定位运行。每到一处，保持 hold 和 RTK Fixed，把 `base_link` 原点的地面投影对准标记，停稳后采样。每个点用不同文件名，后续测试直接复用。

```bash
# 在标记上停车采点
rosrun ugv_navigation geographic.py capture --duration 3 --output "$RUN/mark.json"
printf '目标文件：%s/%s/mark.json\n' "$PWD" "$RUN"
```

也可以用独立 RTK 测量设备提前测地面标记，使用 WGS84 和一致的差分基准。把天线测量中心的地面投影对准标记，记录 Fixed 时的纬经度。飞机端可以用 `ag set --latitude "$LAT" --longitude "$LON"` 提交预先测得的点。

在具有有效配准时，可在测试起点停车后预览路径。`MARK` 填本次采点或历史采点的实际文件路径。

```bash
MARK="$RUN/mark.json"
rosrun ugv_navigation geographic.py plan --target-file "$MARK" --output "$RUN/plan.json"
```

把打印出的 `mark.json` 复制到飞机端操作目录。目标是 **小车参考点**的 WGS84 坐标，不用飞机的实时位置，也不用再补偿天线偏移。

**发令与观察。** 确认路线安全，在小车任务终端执行 `roslaunch ugv_bringup slicer_air_ground.launch`。如需记录，在另一个小车宿主终端先开采集（录包自动结束，或按 Ctrl-C 保存）：

```bash
python3 tools/rtk/collect_odom_test.py --navigation --duration 600 \
  --output "$HOME/workspace/slicer/data/air-ground-record-$(date +%Y%m%d-%H%M%S)"
```

飞机保持静止，在飞机端执行 `ag status` 确认有新鲜状态且 `startup_checked=true`。确认场地和车辆就绪后，飞机端发出单终点任务：

```bash
# 飞机端：用刚复制过来的真实采点文件
ag set --target-file ./mark.json
ag status
```

`accepted=true` 表示任务已受理。看到 `phase=COMPLETED` 后，核对 Action 成功（`state=3`）、`stop.confirmed=true`、`geographic.confirmed=true`，再现场量车体参考点到标记的水平距离。

有余量再发一组新短路线，等车开始导航后，在飞机端分次执行 `ag pause`（等 `PAUSED` 且车停稳）→ `ag resume`（观察继续行驶）→ `ag cancel`（等 `CANCELED` 且停稳）。测试期间由飞机发任务；别在小车上同时用 `goal.py send` 向同一导航节点发令。

**收尾。** 异常时现场急停或人工接管。正常结束后小车操作员执行 `rosrun ugv_slicer_control control.py hold`，确认停稳，再关闭任务、录包和导航终端。收车时关 Motor，再关闭底盘和定位。留存自动准备状态、`mark.json`、测试生成的 `plan.json` 与录包目录。
