#!/usr/bin/env bash
set -euo pipefail
HERE="$(dirname "$(readlink -f "$0")")/combined_gamepad"
export PYTHONDONTWRITEBYTECODE=1
if [[ "${1:-}" == --test ]]; then
    cd "$HERE"
    exec /usr/bin/python3 -B -m unittest -v test_combined_gamepad.py
fi
exec /usr/bin/python3 -s "$HERE/combined_gamepad.py" "$@"
