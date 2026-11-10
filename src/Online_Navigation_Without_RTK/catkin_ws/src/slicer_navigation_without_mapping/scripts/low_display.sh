#!/usr/bin/env bash
# 仅降低桌面像素量，不停止或重启任何机器人节点；不支持该模式时保持现状。
set -eu
command -v xrandr >/dev/null || exit 0
layout=$(xrandr --query)
output=$(printf '%s\n' "$layout" | awk '$2=="connected" && $3=="primary" {print $1;exit}')
if [[ -z "$output" ]]; then output=$(printf '%s\n' "$layout" | awk '$2=="connected" {print $1;exit}'); fi
[[ -n "$output" ]] || exit 0
if printf '%s\n' "$layout" | grep -q '^[[:space:]]*1280x720[[:space:]]'; then
  xrandr --output "$output" --mode 1280x720
fi
