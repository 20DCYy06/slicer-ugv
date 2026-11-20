#!/usr/bin/env bash
set -euo pipefail
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export PYTHONDONTWRITEBYTECODE=1
# 默认入口统一到融合程序，避免旧B停车与新B释放冲突。
# 保留原有离线/只读检查和纯底盘参数接口。
case "${1:-}" in
    --drive-only)
        shift
        exec /usr/bin/python3 -s "$HERE/gamepad.py" "$@"
        ;;
    --demo|--check|--config)
        exec /usr/bin/python3 -s "$HERE/gamepad.py" "$@"
        ;;
    --test)
        exec "$HERE/run_combined_gamepad.sh" --test
        ;;
    -h|--help)
        echo '默认：手柄双摇杆底盘 + Y/A按住舵机顺/逆转 + X吸取/B释放'
        echo '用法：./tools/manual/run_gamepad.sh [--run]'
        echo '      ./tools/manual/run_gamepad.sh --test       无硬件融合测试'
        echo '      ./tools/manual/run_gamepad.sh --drive-only 原纯底盘模式（B停车）'
        echo '      ./tools/manual/run_gamepad.sh --check      原只读底盘检查'
        echo '      ./tools/manual/run_gamepad.sh --demo       原档位演示'
        echo '      ./tools/manual/run_combined_gamepad.sh --run --config FILE 自定义融合配置'
        exit 0
        ;;
esac
if [[ "${1:-}" == --run ]]; then shift; fi
echo '[ENTRY] 融合手柄：X=3吸取 B=1释放 Y=0顺转 A=2逆转；Y/A松开即停'
exec "$HERE/run_combined_gamepad.sh" --run "$@"
