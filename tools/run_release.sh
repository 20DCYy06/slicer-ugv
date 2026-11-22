#!/bin/bash -p
# Install this dispatcher as <slicer-root>/run. It reads the current link once.
set -eo pipefail

usage() {
    printf '%s\n' 'Usage: run {status|rtk [args...]|no-rtk <command> [args...]|shell [command...]}'
}

fail() {
    printf 'Slicer release: %s\n' "$*" >&2
    exit 1
}

mode=${1:-help}
if (( $# )); then shift; fi
case "$mode" in
    help|-h|--help) usage; exit 0 ;;
    status) (( $# == 0 )) || fail 'status takes no arguments' ;;
    rtk|shell) ;;
    no-rtk) [[ ${1:-} != build ]] || fail 'build from a development workspace' ;;
    *) usage >&2; fail "unknown command $mode" ;;
esac

# Load only paths provided by the selected install. Preserve site networking,
# credentials-file selections and desktop session variables from the caller.
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
unset BASH_ENV ENV CDPATH
unset CMAKE_PREFIX_PATH PKG_CONFIG_PATH PYTHONPATH PYTHONHOME LD_LIBRARY_PATH LD_PRELOAD
unset ROS_ROOT ROS_PACKAGE_PATH ROS_ETC_DIR ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION ROSLISP_PACKAGE_DIRECTORIES
unset GIT_DIR GIT_COMMON_DIR GIT_WORK_TREE GIT_OBJECT_DIRECTORY GIT_ALTERNATE_OBJECT_DIRECTORIES GIT_INDEX_FILE
export PYTHONNOUSERSITE=1

root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)
[[ -d "$root/releases" && ! -L "$root/releases" ]] || fail "missing releases directory at $root/releases"
[[ -L "$root/current" ]] || fail "no current release link at $root/current"
release=$(realpath -e -- "$root/current") || fail "cannot resolve $root/current"
[[ -d "$release" && "$(dirname -- "$release")" == "$root/releases" ]] ||
    fail 'current must select a direct directory under releases/'
release_id=${release##*/}
repo="$release/src/slicer-ugv"
overlay="$release/install"
manifest="$release/release.json"

required_path() {
    [[ "$(realpath -e -- "$1" 2>/dev/null)" == "$1" ]] ||
        fail "missing or redirected release path $1"
}

[[ -d "$repo" && -d "$release/runtime/no_rtk" && -f "$overlay/setup.bash" && -f "$manifest" ]] ||
    fail "incomplete release at $release"
for path in "$repo" "$release/runtime/no_rtk" "$overlay/setup.bash" "$manifest"; do
    required_path "$path"
done

manifest_sha=$(python3 - "$manifest" "$release_id" <<'PY'
import json
import re
import sys

manifest, release_id = sys.argv[1:]
try:
    with open(manifest, encoding='utf-8') as stream:
        record = json.load(stream)
    sha = record['vehicle']['sha']
    if record['release_id'] != release_id or not re.fullmatch(r'[0-9a-f]{40}', sha):
        raise ValueError('release ID or vehicle SHA is invalid')
except (KeyError, OSError, TypeError, ValueError) as error:
    sys.exit(f'Cannot read release metadata at {manifest}: {error}')
print(sha)
PY
) || fail 'invalid release manifest'

[[ "$(git -C "$repo" rev-parse --show-toplevel)" == "$repo" &&
   "$(git -C "$repo" rev-parse --verify HEAD)" == "$manifest_sha" ]] ||
    fail 'vehicle checkout commit does not match release manifest'

export SLICER_RELEASE_ID="$release_id" SLICER_RELEASE_WS="$release"
export SLICER_NATIVE_WS="$release" SLICER_NO_RTK_RUNTIME="$release/runtime/no_rtk"
export WORKSPACE2_ROOT="$repo" SLICER_HARDWARE_ROOT="$release/deps/slicer_ugv_hardware"
export SLICER_SITE_DIR="$root/site" SLICER_STATE_DIR="$root/site/state/$release_id"
export SLICER_LOGS_DIR="$root/logs/$release_id" SLICER_RUNTIME_DIR="$root/data/$release_id"
export ROS_HOME="$SLICER_STATE_DIR/ros" ROS_LOG_DIR="$SLICER_LOGS_DIR/ros"

if [[ "$mode" == status ]]; then
    printf 'release_id=%s\nrelease=%s\nvehicle_sha=%s\noverlay=%s\nmanifest=%s\n' \
        "$release_id" "$release" "$manifest_sha" "$overlay" "$manifest"
    printf 'site=%s\nstate=%s\nlogs=%s\ndata=%s\n' \
        "$SLICER_SITE_DIR" "$SLICER_STATE_DIR" "$SLICER_LOGS_DIR" "$SLICER_RUNTIME_DIR"
    exit 0
fi

case "$mode" in
    rtk) entry="$repo/tools/rtk_desktop_ui.sh" ;;
    no-rtk) entry="$repo/tools/no_rtk.sh" ;;
esac
if [[ "$mode" != shell ]]; then
    [[ -f "$entry" ]] || fail "missing release entry $entry"
    required_path "$entry"
fi
mkdir -p -- "$ROS_HOME" "$ROS_LOG_DIR" "$SLICER_RUNTIME_DIR"
source "$overlay/setup.bash"
[[ "${CMAKE_PREFIX_PATH%%:*}" == "$overlay" ]] ||
    fail "selected setup did not activate $overlay"
cd -- "$repo"
if [[ "$mode" == shell ]]; then
    if (( $# )); then exec "$@"; fi
    exec /bin/bash --noprofile --norc
fi
exec /bin/bash "$entry" "$@"
