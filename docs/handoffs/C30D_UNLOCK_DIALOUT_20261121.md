# C30D 解锁命令权限修复 · 2026-11-21

- 从 `fix/c30d-udev-replacement` 的 `0ea2ad4` 在独立工作树修复。现场检出已有无 RTK 参数修改和未跟踪的 `.vscode/`。
- 宿主机账号 `ugv` 已加入 `dialout`，当前终端缺少该进程组。宿主机用 `sg dialout -c 'id -nG'` 验证可加载该组。`--check` 和 `--unlock` 打开归属该组的串口遇到 EACCES 时重入一次，沿用原命令和账户权限。
- `tools/hardware/tests` 的 10 项离线测试通过，覆盖两条命令的权限重试、拒绝重试以及 Motor OFF 清锁条件。宿主机当时没有枚举 C30D `1a86:55d4 / 5CF0016377`，`/dev/c30d` 缺失，实车查询和解锁待现场验证。
- v1.6 由固件按 Motor 输入决定是否接受 HELLO 清锁。当前可在 12 V 仍接通时以 Motor OFF、零 PWM 的状态清锁。Motor ON 清锁需要另行定义固件清锁行为并验证。操作见 [硬件手册](../../Hardware/README.md#查询与解锁)，回退可撤销本次权限修复提交。
