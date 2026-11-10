# 2026-10-31 户外 LIO 实时性与初始化恢复

本次有效工作树为 `.worktrees/online-nav-20261030/slicer_ugv_ros1`，基于 `547fe6b`。用户执行运动与 release/hold；助手仅采集、部署软件、重启定位/导航以及停车后完成已有配准。未启动或重启真实底盘驱动，未发运动目标。

## 原因与最小改动

第一次初始化没有前进：raw MID360 点云 10Hz、IMU 200Hz 且新鲜，FAST-LIO/LIO/scan 虽约9.6Hz，但源时间已落后22～37秒且继续增长。Guard 明确报 `motion needs fresh LIO odometry`，因此保持零速。FAST-LIO 按队列最早数据处理，持续处理稍慢于输入会积压；不是因为没有 release，也没有证据可将本次故障简单归为户外空旷。

只新增 `config/lio_sampling.yaml`，在既有 sensor include 后将 FAST-LIO 的 `/point_filter_num` 从1覆盖为2。用现成抽样参数减小处理量；未改队友源码、blind、体素、投影高度、EKF、Guard或控制器。真实 launch 参数差异测试确认唯一变化就是该参数。

第二次初始化 Action 成功，但客户端最多等待4秒；停车后约2秒才开始完整静止尾段，最终仅确认到1.90秒。新增实际 `GeographicClient.quiet()` 伪时钟回归，并给现有有界等待增加2秒余量：初始化最多6秒，普通 send 最多5秒；连续静止时长、速度/位移、新鲜度判据均不变。不自动重试或插入运动。

## 实际结果

| 同场静止对照 | 抽样1，40秒 | 抽样2，60秒 |
|---|---:|---:|
| LIO频率 | 9.719Hz | 10.000Hz |
| LIO延迟，排除头10秒 | 0.102→0.947秒 | 0.062→0.059秒 |
| LIO最大延迟 | 0.947秒 | 0.0825秒 |
| 静止相对首帧最大位置变化 | 1.19cm | 1.25cm |
| body点云平均点数 | 6715 | 3125 |
| scan平均有限射线数 | 278 | 269 |

这些是现场静止软件观测，不是独立定位精度。FAST-LIO本构建未发布有效匹配特征数，未声称测得该指标。

用户执行 `alignment-02` 的2.5米初始化：Action SUCCEEDED，用户反馈正常、略偏左。运动段LIO 10.000Hz、延迟p95 78ms/最大81ms，scan最大90ms；整包约180秒LIO最大90ms，无继续积压。odom位移2.4977m、LIO2.4975m、轮速里程2.5122m，均为软件估算，未取得尺测距离。

raw→Guard→Mux指令最高均为0.4m/s，MCU目标最高0.4m/s、反馈最高0.427m/s；运动命令均0x16，无fault/失败发送/ready=false，四轮均有持续计数，A轮无长期零计数。

原4秒预算结束时静止尾段1.900秒；同一数据在Action结束后4.063秒首次满足原完整2秒判据。在当前hold、同generation collecting、新鲜RTK及重新完整2秒静止检查通过后，调用现有 `finish_registration` 完成配准，没有重走2.5米。175样本拟合位移2.4983m、RMSE7.9mm；这是拟合内部一致性，不能当作独立地理精度。

随后只将导航切到默认map入口，定位、GNSS、全局EKF和底盘保持运行。只读核验：registered/ready、map全局图40×40m、Mux startup held、最终Twist为零；`map→odom`唯一来源 `/global_localization/ekf`，`odom→base_link`唯一来源 `/slicer_ekf`。最高线速度仍0.4m/s，Guard LIO超时仍0.5秒。

## 验证证据与待办

证据在有效工作树 `.test-results/lio-realtime-20261031/`。原始bag为 `initialize02/rtk-initialize02-20261031/motion.bag`，SHA256：

```text
7bbe60f5ab940f178fbfe51224c5974ea999be51cd26724dd2cb7b452306766b
```

- LIO覆盖候选此前完整517项软件测试通过，ARM构建、21项launch/安装检查通过。
- 加停车等待修复后：189项tools通过；331项Catkin首次有1项既有 `guarded_navigation/test_02_explicit_release_forward_arrival_and_cancel` 失败（模型未收到最终速度）。未改代码/阈值，单独复跑该rostest的2项均通过；初次失败证据保留，精确间歇原因未确认，不能写成全量首次通过。
- 最终本地安装检查通过；ARM重新构建/安装通过，19项地理客户端回归及21项launch检查通过。新延迟停稳回归在原版失败、候选通过；持续运动仍有界失败，状态失效立即失败。
- 尚待独立验证：稀疏后小障碍检测、较长/转弯行程LIO稳定性、真实停车距离与地理返回精度。不要把本次2.5米前进当作绕障或地理到点通过。

## 现场继续

当前已配准且导航为map，不要再 initialize，也不要重启定位。WSL RViz将 `Fixed Frame` 选为 `map`。若用当前停车点作为返回标记，先标记 **base_link原点的地面垂直投影**，保持hold，在现有cup操作shell采点：

```bash
rosrun ugv_navigation geographic.py capture --duration 3 --output evidence/targets/mark-01.json
rosrun ugv_navigation geographic.py plan --target-file evidence/targets/mark-01.json --output evidence/mark-01-plan.json
```

这两条不运动；当前位置plan仅验证文件/接口，真正返回测试应在移动到别处后预规划并再次核对路线。后续运动与停止步骤沿用 [RTK操作单](OUTDOOR_RTK_20261031.md)。文件已存在时改用新名字，采点工具不会覆盖。

## 回退

车端完整回退目录：`/home/potato/slicer_ugv_ros1/.test-results/lio-realtime-20261031/rollback/`。先由现场保持停车并停止**定位和导航**，底盘不需重开，再在potato宿主终端执行：

```bash
python3 /home/potato/slicer_ugv_ros1/.test-results/lio-realtime-20261031/rollback.py
```

它恢复试验前源码与完整install（含旧客户端），不启动节点、不解除保持。之后按操作单重新启动定位及odom初始化；重启定位后不能沿用旧配准。此回退恢复抽样1，已知现场可能再次积压，需重新观察实时性。
