#!/usr/bin/env bash
set -euo pipefail
exec "$(dirname "$(readlink -f "$0")")/run_b15_servo.sh" "$@"
