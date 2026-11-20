#!/usr/bin/env bash
set -euo pipefail
export PYTHONDONTWRITEBYTECODE=1
exec /usr/bin/python3 -s "$(dirname "$(readlink -f "$0")")/b15_servo/keyboard.py" "$@"
