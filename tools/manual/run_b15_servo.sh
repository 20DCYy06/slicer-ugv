#!/usr/bin/env bash
set -euo pipefail
HERE=$(dirname "$(readlink -f "$0")")
export PYTHONDONTWRITEBYTECODE=1
source "$HERE/serial_group.sh"
run_serial_tool /usr/bin/python3 -s "$HERE/b15_servo/keyboard.py" "$@"
