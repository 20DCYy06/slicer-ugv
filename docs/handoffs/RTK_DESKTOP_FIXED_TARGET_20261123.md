# RTK 桌面坐标与测试目标

本任务分支 `feat/rtk-desktop-fixed-target`，工作区
`~/workspace/slicer/dev/rtk-desktop-fixed-target/`。当前实车基线为 RC1
`adb4313b0fb7c825e040fd25b7f1cc61624a26df`。实车 GNSS 重复报文修复是独立提交
`99cfc20`，验收新功能时需确认所选 GNSS 实际采用该修复。

## 界面

桌面在有效 Fixed 时提供单独的 WGS84 天线坐标。通过合格 Fix 配对保留准确的数据
年龄，坐标失效时即时清空。显示的天线坐标可用来标记地面点。界面仍保留已有的
配准后车体位置。测试发点只填纬度和经度，高度沿用提交时的合格 Fix。既有
`geographic.py send` 仍处理自动配准、目标接续和停止。启动、就绪与取消见
[操作手册](../OPERATIONS.md#rtk-本地分屏桌面)。

## 直角障碍复测

现场确认自动配准已完成，RViz 显示目标路径，车辆在直角障碍处原地转向。
该次 RC1 日志 `logs/20261122-workspace-rc1/ros/rtk-navigation-map-z5ok2tua.log`
中有 DWA 局部无路径、Guard 拒绝扫掠车体和全局路径继续更新的记录。
膨胀半径为 0.24 m，车体轮廓采用实车矩形和既有边距。Guard 会独立检查整段扫掠区域。
已有记录缺少事故时刻的原始速度与代价地图，无法分辨自旋的指令来源或格子类型。

使用同一版本、同一起点和目标、同一障碍布置。队友负责现场急停，由记录人员先
启动只读录包，再由现场发送一个目标。复现自旋后及时停车并报告开始时间或
目测位置。先核对 DWA 原始指令、Guard/Mux 输出与实际轮速，再用同一时刻的
局部地图、TF、扫描和两级路径区分障碍占用、未知格和地图边界。

```bash
BASE="$HOME/workspace/slicer"
OUT="$BASE/data/rtk-corner-$(date +%Y%m%d-%H%M%S)"
"$BASE/run" shell python3 tools/rtk/collect_odom_test.py --navigation --duration 180 \
  --output "$OUT" --extra-topic /geographic/navigation_runtime \
  --extra-topic /geographic/prepare/status --extra-topic /geographic/prepare/feedback \
  --extra-topic /geographic/prepare/result
```

预备启动时从车端取得 ROS Master 和 GNSS 部署身份。收到录包启动确认后再发一次
真实 WGS84 目标。关闭录包只停止采集程序。新的实车结果以录包和现场记录为准。


## 验证

整车离线测试 293 项通过。对隔离回环 ROS Master 运行 Qt/RViz 桌面集成检查，
使用模拟 Fix 验证精确配对、失效清空与原有目标行为。证据在任务工作区
`logs/offline-tests.txt`、`logs/isolated-ros-qt-rviz.txt`。这里记录软件验证。
正式 RC1 及队友的现场运行保持原样，等待按发布流程安排新版桌面实车检查。
