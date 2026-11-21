# 视觉抓取 PB14 与 v1.7 安装修复

- 开发从 `a93e040` 起步，使用 `fix/pb14-visual-install` 和独立 Catkin 目录。原工作区保留现场的无 RTK 配置修改和 `.vscode/`。`slicer_noetic_current` 仍指向 v1.6。
- v1.7 的视觉程序此前还在使用带升降点动按钮的旧版本，并导入未安装的 `b15_servo.keyboard`。视觉包现在随包安装 `Hardware/Host/c30d_servo.py`，与组合手柄共用能力查询及脉宽编码。视觉升降要求固件报告 PB14。配置继续复用组合手柄的默认脉宽，Y 上升 2100 µs，A 下降 900 µs，停止 1500 µs，刷新 20 Hz。
- 本次仅更新 `/home/ugv/slicer_noetic_c30d_v17/install` 下的视觉包文件。使用 `source "$HOME/slicer_noetic_c30d_v17/install/setup.bash"` 后执行 `rosrun slicer_visual_grasp run.sh` 预览，追加 `--run` 启用动作。保留用户配置中的吸嘴红点和 10 s 下降时限。操作及参数见 [视觉抓取说明](../../src/slicer_visual_grasp/README.md)。
- 静态检查、独立 Catkin 安装、隔离安装及现场 v1.7 的 `rosrun ... --check` 均通过。两份安装各通过 42 项视觉测试，1 项录制帧回放缺少指定样本而跳过。组合手柄 12 项、独立舵机自检通过。实机只读能力查询返回 `PB14/TIM12_CH1 50Hz neutral=1500us watchdog=500ms`。v1.7 在本地 `:0` 验证三按钮预览和 `YUYV 30.00fps`。本次实机验证限于预览和能力查询。
- 回退时将 `/tmp/slicer-pb14-install-backup.8x1jxJXN` 内 `lib/`、`share/` 下的备份复制到同名安装路径，并按该目录的 `new-files` 清单移除本次新增文件。用户校准在安装目录之外。

## 现场下降时限更新

- 用户配置 `~/.config/slicer_visual_grasp/config.json` 的 `grasp.max_lower_s` 由 `10.0` 调为 `20.0`。v1.7 的 `Runtime.servo_settings()` 读取该值和 `rosrun ... --check` 均已验证。现有操作窗口在关闭并重新启动后读取新值。回退时可将该字段改回 `10.0`。
