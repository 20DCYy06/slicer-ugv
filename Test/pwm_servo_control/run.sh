#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$(readlink -f "$0")")"
export PYTHONDONTWRITEBYTECODE=1
mode="${1:-}"
if [[ "$mode" == --check ]]; then
    exec sudo /usr/bin/python3 -s hid_bridge.py --check
fi
# SSH终端默认显示在Jetson本机远程桌面；--x11保留调用者的X11转发。
if [[ "$mode" == --x11 ]]; then
    shift
else
    export DISPLAY="${PWM_DESKTOP_DISPLAY:-:0}"
    auth="/run/user/$(id -u)/gdm/Xauthority"
    if [[ -r "$auth" ]]; then export XAUTHORITY="$auth";
    elif [[ -r "$HOME/.Xauthority" ]]; then export XAUTHORITY="$HOME/.Xauthority"; fi
fi
background=false
if [[ "${1:-}" == --background ]]; then background=true; shift; fi
if [[ " $* " != *' --demo '* && " $* " != *' --self-test '* ]]; then
    need_sudo=$(/usr/bin/python3 -s -c 'from hid_bridge import boards; import os; p=boards(); print(int(len(p)==1 and not os.access(p[0],os.R_OK|os.W_OK)))')
    if [[ "$need_sudo" == 1 ]]; then
        echo '仅HID连接助手需要权限；图形界面仍以potato运行。'
        sudo -v
    fi
fi
if "$background"; then
    mkdir -p logs
    nohup /usr/bin/python3 -s -u ui.py "$@" >logs/ui.log 2>&1 </dev/null &
    echo "已请求桌面窗口，PID=$!；日志：$PWD/logs/ui.log"
else
    exec /usr/bin/python3 -s -u ui.py "$@"
fi
