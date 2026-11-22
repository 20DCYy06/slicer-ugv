# 视觉反馈对准与抓取

```bash
source "$HOME/slicer_noetic_c30d_v17/install/setup.bash"
rosrun slicer_visual_grasp run.sh
rosrun slicer_visual_grasp run.sh --run
```

默认在 Jetson 本地桌面预览，使用 `--run` 启用三个动作按钮。启用前先退出占用底盘或 AirGrip 串口的程序。启动器会复用账号已有的 `dialout` 权限。

升降使用 C30D 的 PB14（TIM12_CH1）。下发升降脉宽前，程序查询固件确认输出引脚。默认脉宽和组合手柄一致，Y 对应上升 2100 µs，A 对应下降 900 µs，停止回中 1500 µs，指令刷新 20 Hz。

- **移动并抓取**：先确认 AirGrip 连通且吸口空闲。移动期间每秒用新帧计算红点到绿框中心的偏差。每个校正短段最多 2 cm，进行中的转向和直行保持同一个方向。停稳后再取新帧，以新偏差决定下一短段。丢框时用编码器和 IMU 扣除已走位移，再走完最后的视觉目标。只有新帧确认距离达标或完成剩余位移后复核无绿框，才进入抓取。确认开泵后下放吸嘴，吸附后保持抽气并抬升 2 秒。
- **释放**：确认关泵后回升 `max(0, t − 2)` 秒并回中。若抬升被打断，释放只补足本窗口记录的剩余时间。下降时间小于或等于 2 秒且抬升已完成时，释放只关泵。
- **立即停止**：取消当前移动或升降，同时请求关闭气泵。Esc / 空格执行相同操作。确认情况见日志。

时间 `t` 是舵机指令维持时间，仅记录在当前窗口内。没有记录时，释放只关泵。右键点击画面中的吸嘴中心可以保存红点，R 重连相机。启动和预览不占用底盘或气泵串口。

RTK 桌面使用 `rosrun slicer_visual_grasp run.sh --run --headless --pick-on-start`。桌面等待后台相机首次可用，随后和可执行窗口调用相同的正式对准、下放、吸附和抬升流程。后续通过同一进程的标准输入发送 `release` 或 `pick`，每行一条。省略 `--pick-on-start` 时等待指令，可用于仅释放。`GRASP_STATE` 后的 `held` 表示抓取和抬升完成，`idle` 表示待命或释放完成，`recovery` 表示需要释放或处理剩余行程，`error` 表示操作失败，`busy` 表示等待或执行中。

无窗口模式把持物、关泵和剩余升降时间保存在配置目录下的 `data/grasp_recovery.json`。关闭进程不会自动松开已确认抓住的物体。正常中断后，重启会报告 `recovery`，释放时先确认关泵，再补足已有记录的回升。若升降中被强制终止或无法读取恢复记录，释放只确认关泵并保持 `recovery`。关闭抓取程序并在现场手动恢复吸嘴初始高度后，清除该记录再启动抓取。

## 配置

包内默认值见 [config/config.json](config/config.json)。右键校准保存到 `~/.config/slicer_visual_grasp/config.json`，设置了 `XDG_CONFIG_HOME` 时使用该目录。随后启动会读取该用户配置。也可用 `--config /path/to/config.json` 指定并编辑现有配置。

设备默认使用随包提供的底盘、AirGrip 和舵机配置。`hardware_config`、`airgrip_config`、`servo_config` 可指定绝对路径或相对当前配置文件的路径。升降方向由 `servo_up_direction` 指定，当前为 `clockwise`。最大下降时长由 `grasp.max_lower_s`（秒）指定，未配置时沿用独立舵机的点动时长。固定红点是正式模式。模板模式读取相对当前配置文件目录的 `target.nozzle.template_dir`，默认是 `data/nozzle_templates`。

```json
"snapshot_move": {
  "surface_size_m": [0.10, 0.07],
  "forward_image_direction": "down"
}
```

- `alignment.tolerance_px`：红绿中心的欧氏距离阈值，默认 8 像素。连续 3 个短段停稳后，相对最好的视觉误差累计都没有减少至少 2 像素时，停止并报未收敛。对准与复核的总时限为 60 秒。相机错误或画面过期会中止。
- `target.nozzle.origin_px`：固定红点，即吸嘴在目标平面的实际落点；相机位置变化后需校准。
- 底盘近距离速度上限为 0.05 m/s，角速度上限为 0.20 rad/s。位移只由视觉换算得到。

## 日志与检查

`[VISION_UPDATE]` 记录每次计算的像素误差和位移向量，`[VISION_STEP]` 记录实际发起的短段或丢框后的剩余位移，`[VISION_ALIGNED]` 记录停稳后的复核结论。
最后一次计算的移动计划和对应画面保存在当前配置目录的 `data/last_move_plan.json` 和 `data/last_move_snapshot.jpg`。
`[STATE] LIFT_AFTER_GRASP` 记录吸附后的下降时间和抬升计划。抓取的 `[DONE]` 记录抬升结束及剩余回升时间，释放的 `[DONE]` 表示回升结束。
`[STOP_PUMP]` / `[STOP_PUMP_ERROR]` 显示关泵结果。

```bash
rosrun slicer_visual_grasp run.sh --check
rosrun slicer_visual_grasp run.sh --test
rosrun slicer_visual_grasp run.sh --ui-test
```

测试使用假相机、假底盘与假气泵。界面回归需要 `xvfb-run` 和 `xauth`。如需复放已有的 `fixed_origin_current.jpg`，为 `--test` 命令设置 `VISUAL_GRASP_TEST_FRAME` 指向该图像。

从根目录也可直接执行 `./src/slicer_visual_grasp/run.sh`。归档的旧视觉调试程序 `Test/Vision_adjust` 可用 `git show c80b68b436ec18ff4a31bb625f99a007239cccdf:Test/Vision_adjust/README.md` 查阅。
