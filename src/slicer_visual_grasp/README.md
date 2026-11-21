# 单次视觉位移 + 3 cm 靠近补偿 + 开泵

```bash
source "$HOME/slicer_noetic_c30d_v17/install/setup.bash"
rosrun slicer_visual_grasp run.sh
rosrun slicer_visual_grasp run.sh --run
```

默认在 Jetson 本地桌面预览，使用 `--run` 启用三个动作按钮。启用前先退出占用底盘或 AirGrip 串口的程序。启动器会复用账号已有的 `dialout` 权限。

升降使用 C30D 的 PB14（TIM12_CH1）。下发升降脉宽前，程序查询固件确认输出引脚。默认脉宽和组合手柄一致，Y 对应上升 2100 µs，A 对应下降 900 µs，停止回中 1500 µs，指令刷新 20 Hz。

- **移动并抓取**：以当前相机红点为吸嘴原点，锁定目标并移动底盘。停稳且释放底盘串口后开启抽气，收到本次开泵反馈才下放吸嘴。新鲜反馈确认吸附时回中并记录下降时间 `t`。下降到时限仍未吸附或反馈中断时回中并请求关泵。
- **释放**：确认关泵后，按本窗口记录的下降时间回升并回中。如先前的回升被打断，下一次释放使用剩余时间。
- **立即停止**：取消当前移动或升降，同时请求关闭气泵。Esc / 空格执行相同操作。确认情况见日志。

时间 `t` 是舵机指令维持时间，仅记录在当前窗口内。没有记录时，释放只关泵。右键点击画面中的吸嘴中心可以保存红点，R 重连相机。启动和预览不占用底盘或气泵串口。

## 配置

包内默认值见 [config/config.json](config/config.json)。右键校准保存到 `~/.config/slicer_visual_grasp/config.json`，设置了 `XDG_CONFIG_HOME` 时使用该目录。随后启动会读取该用户配置。也可用 `--config /path/to/config.json` 指定并编辑现有配置。

设备默认使用随包提供的底盘、AirGrip 和舵机配置。`hardware_config`、`airgrip_config`、`servo_config` 可指定绝对路径或相对当前配置文件的路径。升降方向由 `servo_up_direction` 指定，当前为 `clockwise`。最大下降时长由 `grasp.max_lower_s`（秒）指定，未配置时沿用独立舵机的点动时长。固定红点是正式模式。模板模式读取相对当前配置文件目录的 `target.nozzle.template_dir`，默认是 `data/nozzle_templates`。

```json
"snapshot_move": {
  "approach_extra_m": 0.03,
  "surface_size_m": [0.10, 0.07],
  "forward_image_direction": "down"
}
```

- `approach_extra_m`：沿锁定向量的额外距离，单位米；设为0关闭补偿。
- `target.nozzle.origin_px`：固定红点，即吸嘴在目标平面的实际落点；相机位置变化后需校准。
- 底盘近距离速度上限保持0.05 m/s、角速度上限0.20 rad/s，未改固件、手柄或其他程序。

3 cm是对本次现场实测误差的固定补偿，并不等于完成相机/TCP几何标定。实际位移仍有原有到达容差和轮胎打滑误差。

## 日志与检查

`[SNAPSHOT_LOCKED]` 同时打印原始视觉XY、额外距离和最终XY。
执行任务后，计划与起始原图保存在当前配置目录的 `data/last_move_plan.json` 和 `data/last_move_snapshot.jpg`。
抓取的 `[DONE]` 记录已确认吸附及下降时间；释放的 `[DONE]` 表示回升结束。
`[STOP_PUMP]` / `[STOP_PUMP_ERROR]` 显示关泵结果。

```bash
rosrun slicer_visual_grasp run.sh --check
rosrun slicer_visual_grasp run.sh --test
rosrun slicer_visual_grasp run.sh --ui-test
```

测试使用假相机、假底盘与假气泵。界面回归需要 `xvfb-run` 和 `xauth`。如需复放已有的 `fixed_origin_current.jpg`，为 `--test` 命令设置 `VISUAL_GRASP_TEST_FRAME` 指向该图像。

从根目录也可直接执行 `./src/slicer_visual_grasp/run.sh`。归档的旧视觉调试程序 `Test/Vision_adjust` 可用 `git show c80b68b436ec18ff4a31bb625f99a007239cccdf:Test/Vision_adjust/README.md` 查阅。
