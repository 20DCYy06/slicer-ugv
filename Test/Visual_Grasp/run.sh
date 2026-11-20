#!/usr/bin/env bash
set -euo pipefail
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export PYTHONDONTWRITEBYTECODE=1
# 不加载用户目录NumPy2，避免与Ubuntu OpenCV4.5发生ABI冲突。
if [[ "${1:-}" == --test ]]; then
    cd "$HERE"
    exec /usr/bin/python3 -s -m unittest -v test_visual_grasp test_two_point test_fixed_origin test_snapshot_move
fi
if [[ "${1:-}" == --ui-test ]]; then
    cd "$HERE"
    exec xvfb-run -a -s '-screen 0 800x600x24' /usr/bin/python3 -s -m unittest -v test_ui_resilience
fi
if [[ "${1:-}" != --check && "${1:-}" != --help && "${1:-}" != -h ]]; then
    # 使用Jetson本地桌面；不继承SSH的localhost:10.0，也不使用sudo/xhost放开权限。
    export DISPLAY="${VISUAL_GRASP_DISPLAY:-:0}"
    auth="/run/user/$(id -u)/gdm/Xauthority"
    if [[ ! -r "$auth" ]]; then auth="$HOME/.Xauthority"; fi
    if [[ ! -r "$auth" ]]; then
        echo '未找到本地桌面认证，请先登录Jetson图形桌面。' >&2
        exit 1
    fi
    export XAUTHORITY="$auth"
fi
exec /usr/bin/python3 -s "$HERE/app.py" "$@"
