#!/usr/bin/env bash
set -eo pipefail

if [[ ${1:-} == --help || ${1:-} == -h ]]; then
  printf 'Usage: %s\nOpens local X11 RViz and RTK workflow controls on ROS_MASTER_URI.\n' "$0"
  exit 0
fi
if [[ $# != 0 ]]; then
  printf 'Run %s without arguments. Use --help for usage.\n' "$0" >&2
  exit 2
fi

repo=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)
workspace=$(realpath -m -- "${SLICER_NATIVE_WS:-${repo%/src/slicer-ugv}}")
if [[ "$repo" != "$workspace/src/slicer-ugv" ]]; then
  printf 'The entrypoint checkout must be %s/src/slicer-ugv.\n' "$workspace" >&2
  exit 2
fi
setup="$workspace/install/setup.bash"
if [[ ! -r "$setup" ]]; then
  printf 'No installed ROS overlay at %s. Run tools/build_native.sh for this workspace.\n' "$workspace" >&2
  exit 2
fi
source /opt/ros/noetic/setup.bash
source "$setup"
if [[ $(rospack find ugv_navigation) != "$workspace/install/share/ugv_navigation" ]]; then
  printf 'Wrong ROS overlay for ugv_navigation. Expected the installation in %s.\n' "$workspace" >&2
  exit 2
fi
export SLICER_RUNTIME_DIR="${SLICER_RUNTIME_DIR:-$workspace/runtime}"
export ROS_HOME="${ROS_HOME:-$workspace/logs/ros}"

if ! [[ ${DISPLAY:-} =~ ^:[0-9]+(\.[0-9]+)?$ ]] || ! xset q >/dev/null 2>&1; then
  desktop_runtime="/run/user/$UID"
  while IFS='=' read -r name value; do
    case "$name" in
      DISPLAY) local_display=$value ;;
      XAUTHORITY) local_authority=$value ;;
    esac
  done < <(XDG_RUNTIME_DIR="$desktop_runtime" DBUS_SESSION_BUS_ADDRESS="unix:path=$desktop_runtime/bus" \
           systemctl --user show-environment 2>/dev/null)
  if ! [[ ${local_display:-} =~ ^:[0-9]+(\.[0-9]+)?$ ]]; then
    printf 'Log in to the Jetson local X11 desktop before opening RTK UI.\n' >&2
    exit 2
  fi
  export DISPLAY="$local_display" XDG_RUNTIME_DIR="$desktop_runtime"
  export XAUTHORITY="${local_authority:-$HOME/.Xauthority}"
  if ! xset q >/dev/null 2>&1; then
    printf 'The Jetson local X11 session is not accessible to this account.\n' >&2
    exit 2
  fi
fi

export QT_QPA_PLATFORM=xcb
export ROS_NAMESPACE="/slicer_rtk_desktop_$$"
export SLICER_NATIVE_WS="$workspace"
export PYTHONPATH="$workspace/install/lib/ugv_navigation${PYTHONPATH:+:$PYTHONPATH}"
exec python3 -s -B "$repo/tools/rtk/desktop/desktop.py" "$@"
