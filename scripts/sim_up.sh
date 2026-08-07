#!/usr/bin/env bash
set -euo pipefail

mode="${1:-gui}"
if [[ "${mode}" == "headless" ]]; then
  exec docker run --rm --name unitree-g1-gazebo --network host \
    unitree-g1-gazebo:jazzy \
    ros2 launch g1_gazebo sim.launch.py headless:=true rviz:=false
fi

xhost +si:localuser:root >/dev/null
cleanup_xhost() {
  xhost -si:localuser:root >/dev/null 2>&1 || true
}
trap cleanup_xhost EXIT

docker run --rm --name unitree-g1-gazebo --network host \
  -e DISPLAY="${DISPLAY:-:0}" \
  -e QT_X11_NO_MITSHM=1 \
  -e LIBGL_ALWAYS_SOFTWARE=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  unitree-g1-gazebo:jazzy \
  ros2 launch g1_gazebo sim.launch.py headless:=false rviz:=true
