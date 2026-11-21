#!/usr/bin/env bash
set -euo pipefail
HERE=$(dirname "$(readlink -f "$0")")
export PYTHONDONTWRITEBYTECODE=1
if [[ "${1:-}" == --test ]]; then
    cd "$HERE/combined_gamepad"
    exec /usr/bin/python3 -B -m unittest -v test_combined_gamepad.py
fi
source "$HERE/serial_group.sh"
run_serial_tool /usr/bin/python3 -s "$HERE/combined_gamepad/combined_gamepad.py" "$@"
