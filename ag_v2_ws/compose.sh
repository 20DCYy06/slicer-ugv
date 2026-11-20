#!/usr/bin/env bash
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec docker --config "$root/.docker" compose --env-file "$root/bridge.env" \
  -f "$root/src/AG_Client/deploy/ugv/compose.yaml" "$@"
