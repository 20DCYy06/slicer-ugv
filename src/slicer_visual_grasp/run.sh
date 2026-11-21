#!/usr/bin/env bash
set -euo pipefail
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ -f "$HERE/package.xml" ]]; then
    export PYTHONPATH="$HERE/src${PYTHONPATH:+:$PYTHONPATH}"
    serial_tools="$HERE/../../tools/manual"
else
    HERE="$(rospack find slicer_visual_grasp)"
    serial_tools="$HERE/support/tools/manual"
fi
export PYTHONDONTWRITEBYTECODE=1
# 不加载用户目录NumPy2，避免与Ubuntu OpenCV4.5发生ABI冲突。
case "${1:-}" in
    --test)
        cd "$HERE/tests"
        exec /usr/bin/python3 -s -m unittest -v test_visual_grasp test_two_point test_fixed_origin test_snapshot_move
        ;;
    --ui-test)
        cd "$HERE/tests"
        exec xvfb-run -a -s '-screen 0 800x600x24' /usr/bin/python3 -s -m unittest -v test_ui_resilience
        ;;
esac

enable_actions=false
for argument in "$@"; do
    case "$argument" in
        --check|--help|-h) exec /usr/bin/python3 -s -m slicer_visual_grasp.app "$@" ;;
        --run) enable_actions=true ;;
    esac
done

# 使用Jetson本地桌面；不继承SSH的localhost:10.0，也不使用sudo/xhost放开权限。
export DISPLAY="${VISUAL_GRASP_DISPLAY:-:0}"
auth="/run/user/$(id -u)/gdm/Xauthority"
if [[ ! -r "$auth" ]]; then auth="$HOME/.Xauthority"; fi
if [[ ! -r "$auth" ]]; then
    echo '未找到本地桌面认证，请先登录Jetson图形桌面。' >&2
    exit 1
fi
export XAUTHORITY="$auth"

if [[ "$enable_actions" == true ]]; then
    source "$serial_tools/serial_group.sh"
    run_serial_tool /usr/bin/python3 -s -m slicer_visual_grasp.app "$@"
fi
exec /usr/bin/python3 -s -m slicer_visual_grasp.app "$@"
