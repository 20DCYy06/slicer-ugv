#!/usr/bin/env bash
# Jetson: RGB acquisition only. No chassis, navigation goal or control release.
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
config="$root/src/Online_Navigation_with_RTK/src/ugv_bringup/config/vision_camera.yaml"
read -r usb_serial sdk_serial width height fps < <(python3 -s - "$config" <<'PY'
import sys,yaml
c=yaml.safe_load(open(sys.argv[1]))
print(c['usb_serial'],c['sdk_serial'],c['width'],c['height'],c['fps'])
PY
)
nodes="$(python3 -s "$root/src/Online_Navigation_with_RTK/tools/camera_devices.py" "$usb_serial")"
devices=()
while IFS= read -r node; do devices+=(--device "$node"); done <<< "$nodes"
if docker inspect slicer-vision-camera >/dev/null 2>&1; then
 if [[ "$(docker inspect -f '{{.State.Running}}' slicer-vision-camera)" == true ]]; then
  mapped_nodes="$(docker inspect -f '{{range .HostConfig.Devices}}{{println .PathOnHost}}{{end}}' slicer-vision-camera | sort)"
  if [[ "$mapped_nodes" == "$(printf '%s\n' "$nodes" | sort)" ]]; then
   echo 'slicer-vision-camera is already running'; exit 0
  fi
  docker stop slicer-vision-camera >/dev/null
 fi
 docker rm slicer-vision-camera >/dev/null
fi
ros_ip="${ROS_IP:-$(ip -4 route get 1.1.1.1 2>/dev/null | awk '{for(i=1;i<NF;i++)if($i=="src"){print $(i+1);exit}}' || true)}"
ros_ip="${ros_ip:-127.0.0.1}"
# Only the selected camera nodes are exposed; host USB permissions stay unchanged.
exec docker run -d --name slicer-vision-camera --network host --cap-drop ALL \
 --security-opt no-new-privileges --tmpfs /tmp:rw,mode=1777 "${devices[@]}" \
 --mount type=bind,src=/run/udev,dst=/run/udev,readonly \
 -e "ROS_MASTER_URI=${ROS_MASTER_URI:-http://127.0.0.1:11311}" -e "ROS_IP=$ros_ip" \
 -e ROS_HOME=/tmp/slicer-vision-ros \
 --entrypoint /bin/bash ugv-online-rtk:arm64-20261030 -c \
 'unset ROS_HOSTNAME; source /opt/ros/noetic/setup.bash; exec roslaunch realsense2_camera rs_camera.launch serial_no:="$1" enable_color:=true enable_depth:=false enable_infra1:=false enable_infra2:=false enable_gyro:=false enable_accel:=false color_width:="$2" color_height:="$3" color_fps:="$4"' \
 bash "$sdk_serial" "$width" "$height" "$fps"
