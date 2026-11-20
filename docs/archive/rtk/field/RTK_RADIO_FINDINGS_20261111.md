# 2026-11-11 户外RTK：安装位置干扰对照与收车记录

**今天没有执行 initialize、非零导航或地理返回测试。** 主要进展是取得接收机射频诊断，并在抬高位置恢复一次实际RTK Fixed；该结果只证明静态接收可用，不是新底盘户外导航通过。用户决定收车，结构调整与后续运动留待下一轮。

接收机串口只读回报为 **UM982 / R4.10Build7980**。本轮差分认证与回注可工作；默认datum为WGS84，服务地址 `103.143.19.54:8002`，挂载点 `RTCM33GRCEJpro`。凭据只留车端受保护文件，本文与结果包不记录密码。试过备用32挂载点后恢复33，没有以放宽Fixed质量规则替代接收问题。

**实际对照结果**（AGC列为主链路L1/L2/L5；JAM是接收机报告，不是Guard判定）：

| 状态 | JAM flag / CWRatio | 主AGC | 能说明什么 |
|---|---|---|---|
| 原安装位、接车端USB | 2 / 123 | 2 / 1 / 44 | 接收机报告强窄带干扰，主L1/L2增益很低 |
| 原位、接车端USB，仅关MID360供电 | 2、2、2 / 195、200、146 | 3 / 2 / 43 | 雷达关闭后问题仍存在，不支持其为唯一必要来源 |
| 接笔记本USB，同时移动接收机 | 0、0、0 / 10、12、17 | 49 / 49 / 64 | 位置与USB供电同时变化，不能归因纯供电 |
| 仍接笔记本USB，放回原安装位 | 2、2 / 255、249 | 31–33 / 30–33 / 43–44 | 原位重新出现强干扰；不能仅归因Jetson USB |
| 仍接笔记本USB，垂直抬高约30–50cm | 1、1、1 / 53、61、67 | 44–45 / 45–46 / 58–59 | 干扰减弱，但仍是“检测到干扰”，不是无干扰 |
| 接回车端USB、手持抬高，Motor OFF | 实际q=4、18星、差分龄约2s、fresh_rtk_usable=true | 未将同期AGC写成已采到 | 静态Fixed恢复，未执行运动或配准 |

笔记本原位阶段共收到两条JAM；另一个样本只有AGC，不能报成三次JAM。最初 `radio-windows-usb.json` 中“位置未变”的note已被用户补充及 `placement-notes.json` 明确纠正。手持、朝向、线缆形状和采样时刻不完全受控，以上是现场对照，不能当实验室单因定论。

**结论与界限。** 当前证据支持安装位置、局部电磁环境或连接耦合影响接收；尚不能区分辐射拾取与供电/地线/USB传导路径，不能锁定单一干扰源，也没有证实天线损坏。短时查询曾主动暂停RTCM，因此当时RTKSTATUS“无差分/延迟”原因不能倒推为原故障；AGC/JAM仍可用于该时段射频观察。`HWFlag=0x84`不是天线开短标志。CW flag 0/1/2只作诊断证据，**不新增“必须CW=0才能导航”的门禁**。

**最终安装与本轮收尾。** 用户随后将一体接收机改到雷达后约20cm、估计左右居中、远离金属的位置；20cm不是精确尺测。根据雷达在base_link前0.065m，`gnss_site.yaml`已以注释保留估计依据，水平杆臂改为 **[-0.135, 0]m**，z仍null，沿用二维补偿。父任务已将车端source/install同步。当前函数加载负杆臂及四朝向正逆变换在无设备、network none容器通过，误差小于1e-12；没有修改生产控制代码或新增测试框架。

最终车尾安装位的45秒静态watch **全程q=2、10–12星、HDOP1.8–2.2**，差分龄约2秒、RTCM写入年龄0.006–0.38秒，结果为 **NOT_FIXED_YET：本次固定安装位Fixed复核未通过**。不能沿用手持抬高的q4宣布最终安装通过，也不能因差分字节持续回注就认定接收机已完成RTK固定。

本轮 `outdoor-01/capture` 约15分钟录包已正常关闭，大小 **51,142,883 bytes、finalized=true**，属于静态/准备采集，不能称为运动成功。已取回本地并核对车端/本地整包SHA256；manifest同时确认录包订阅建立、退出码0、正常封包。本轮RViz、导航、定位和GNSS后台已停止。已有视觉伺服工作区修改保留，未合并、回滚或覆盖。仅另改了本地RViz观测默认master地址；车端运行网络配置未因本机显示而改动。

**本机RViz准备。** WSL到master、无设备观察容器到车端odom/LIO/scan/GNSS节点RPC、车端到观察容器动态端口，三段网络实测通过；本轮未打开GUI，画面渲染未目测。未修改路由或防火墙。下次地址仍相同时，在本机WSL的rescue终端手动打开：

```bash
cd /home/rescue/slicer_ugv_ros1/current/src/Online_Navigation_with_RTK
ROS_IP=192.168.43.228 bash tools/rviz_wsl.sh navigation 192.168.43.187
```

这是odom观察；配准并切map导航后可把窗口Fixed Frame改为map。网络地址变化时重新核对，不照搬旧电台或旧热点地址。

**结构研究只保留四项待办：**

1. 先将接收机固定在候选位置，保持明确朝向与天线水平杆臂；比较原位、后移、抬高时的JAM/AGC与正常RTCM下的Fixed稳定性，避免手持结果替代最终安装。
2. 在固定位置后逐个检查邻近电子设备、供电模块、USB/其他线缆的距离和走线；每次只改变一个因素，找出可复现变化，再决定结构调整，不先判某个USB电源或雷达损坏。
3. 最终装配、雷达及正常电子设备全部工作时做静态接收复核，保留Fixed/Float、卫星/HDOP、差分龄和AGC/JAM记录；不通过放宽质量或安全规则获得“通过”。
4. 静态状态可靠后，复用[本轮操作单](OUTDOOR_RTK_20261111.md)：前进初始化2.5m、停车切map、采新标记、首次3–5m返回，并用同一base_link地面参考点尺测。该运动验收今天未开始。

**本地证据索引。** 根目录 `/home/rescue/slicer_ugv_ros1/.tmp/rtk-20261111-01/`；以下均为相对路径，不含凭据：

- `outdoor-01/raw-gnss-check.json`、`raw-gnss-summary.json`：初期NMEA、使用卫星和多信号去重；早期9星样本不与后来的18星混算。
- `outdoor-01/um982-query.json`、`um982-status-query.json`：实际身份、配置查询和诊断原文，标有RTCM主动暂停。
- `outdoor-01/radio-radar-off.json`、`radio-windows-usb.json`、`radio-windows-original-position.json`、`radio-windows-raised-position.json`：各阶段原文及主/从AGC、JAM；后续三个文件名同样位于outdoor-01下。
- `outdoor-01/placement-notes.json`、`radio-placement-comparison.json`：现场位置纠正、逐阶段事实/推断和SHA256。
- `outdoor-01/negative-lever-check.json`：当前杆臂配置及函数哈希、四朝向软件验证，非实测尺寸或导航验收。

字段解释参考[Unicore官方N4指令手册R1.4](https://en.unicore.com/uploads/file/20241219/Unicore_Reference_Commands_Manual_For_N4_High_Precision_Products_V2_EN_R1.4.pdf)；旧Build7980未确认的新增字段不套用新版含义。
