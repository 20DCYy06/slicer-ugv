# 单次视觉位移 + 3 cm 靠近补偿 + 开泵

```bash
source "$HOME/slicer_noetic_current/install/setup.bash"
rosrun slicer_visual_grasp run.sh
rosrun slicer_visual_grasp run.sh --run
```

默认在 Jetson 本地桌面预览，点 **启用操作** 或用 `--run` 开启动作按钮。启用前先退出 ROS 底盘驱动、独立手柄和 AirGrip 控制程序，将底盘与气泵串口留给本程序。启动器会复用账号已有的 `dialout` 权限。

动作按钮为 **移动并抓取 / 释放 / 停止抽气 / 上升 / 下降**。按住升降按钮点动，松手、移出按钮或窗口失焦即停止。每次点动受舵机配置的时长限制，默认 0.3 秒。上升 2100 µs、下降 900 µs、中位 1500 µs。抓取与升降共用底盘串口，按顺序执行。自动抓取仍只移动底盘并开泵。

## 执行流程

1. 点击“移动并抓取”，下发零速并等待底盘反馈静止。
2. 读取一次图像中的粉色物块中心、物块四角和固定红点原点。
3. 用当前可见面的7×10 cm尺寸换算底盘XY，在该向量方向上额外加3 cm，并锁定最终目标。
4. 通过原有编码器线速度及IMU角速度反馈执行转向、移动、恢复初始朝向。
5. 停稳后开启气泵，确认本次开泵指令及泵运行状态后结束；吸附状态打印到日志，不冒充已经吸住。

运动过程中不重新读取图像用于控制；后期夹具遮挡导致检测框消失，不会取消已锁定的位移。
画面仍可持续预览。3 cm补偿合入一次性目标，没有增加第二次识别或重复靠近循环。
补偿方向是静止帧中红点→绿点换算后的方向，不是固定底盘正方向。
若红绿点重合而无方向，则保持零位移，不猜测额外方向。

## 停止抽气与释放

“停止抽气”在移动任务执行期间仍可点击：先中断底盘任务，等待旧连接释放，再独占连接AirGrip发送PUMP_OFF。
该过程在工作线程中执行，不阻塞界面，也不与原任务争抢串口。
“释放”在空闲时同样发送PUMP_OFF。现有接口没有在本程序中实现独立的“停泵且保持负压”动作，因此停止抽气可能释放物块。
Esc/空格只停止底盘任务；退出仍保持当前气泵状态，不自动释放。
Ctrl+C、窗口关闭、相机重连和单实例逻辑沿用现有实现。R 重连相机。点击 **校准红点** 后在画面点选吸嘴中心，或直接右键点选。

## 配置

包内默认值见 [config/config.json](config/config.json)。右键校准保存到 `~/.config/slicer_visual_grasp/config.json`，设置了 `XDG_CONFIG_HOME` 时使用该目录。随后启动会读取该用户配置。也可用 `--config /path/to/config.json` 指定并编辑现有配置。

设备默认使用随包提供的底盘、AirGrip 和舵机配置。`hardware_config`、`airgrip_config`、`servo_config` 可指定绝对路径或相对当前配置文件的路径。升降方向由 `servo_up_direction` 指定，当前为 `clockwise`。固定红点是正式模式。模板模式读取相对当前配置文件目录的 `target.nozzle.template_dir`，默认是 `data/nozzle_templates`。

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
`[DONE]` 表示移动结束且开泵得到确认；是否已经吸住以打印的吸附状态为准。
`[STOP_PUMP]` / `[STOP_PUMP_ERROR]` 显示关泵结果。

```bash
rosrun slicer_visual_grasp run.sh --check
rosrun slicer_visual_grasp run.sh --test
rosrun slicer_visual_grasp run.sh --ui-test
```

测试使用假相机、假底盘与假气泵。界面回归需要 `xvfb-run` 和 `xauth`。如需复放已有的 `fixed_origin_current.jpg`，为 `--test` 命令设置 `VISUAL_GRASP_TEST_FRAME` 指向该图像。

从根目录也可直接执行 `./src/slicer_visual_grasp/run.sh`。归档的旧视觉调试程序 `Test/Vision_adjust` 可用 `git show c80b68b436ec18ff4a31bb625f99a007239cccdf:Test/Vision_adjust/README.md` 查阅。
