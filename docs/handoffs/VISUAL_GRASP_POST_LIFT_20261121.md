# 视觉抓取吸附后抬升

- 从 `5753c91` 在 `fix/visual-grasp-post-pick-lift` 的独立工作区开发。原工作区的无 RTK 配置修改和 `.vscode/` 保留。现场更新仅覆盖 v1.7 安装中的 `pipeline.py` 和 `test_snapshot_move.py`。
- 确认吸附后保持抽气，用 PB14 抬升 2 s 并回中。释放先关泵，再补足下降时间 `t` 减去已抬升的时间。抬升完成且 `t ≤ 2 s` 时，释放只关泵。抬升中断时按实际发送升降指令的时间计算剩余回升量。持物状态也会单独记录，直到确认关泵。
- 独立 Catkin 安装和现场 v1.7 安装均通过 `run.sh --check`，各通过 42 项离线测试，缺少样本的录制帧回放跳过。现场配置验证为下降超时 `20 s`，吸附后抬升 `2 s`。未发出硬件指令。下次使用 v1.7 启动时生效，观察 `[STATE] LIFT_AFTER_GRASP` 和 `[DONE]` 日志。详见 [视觉抓取说明](../../src/slicer_visual_grasp/README.md)。
- 回退时将 `/tmp/slicer-post-grasp-lift-backup.pGGzvTFm` 下的两个文件复制回 v1.7 安装中的对应 `lib/`、`share/` 路径。
