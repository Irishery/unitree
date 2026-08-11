#!/usr/bin/env bash
set -euo pipefail

xhost +si:localuser:root >/dev/null
docker run --rm --name unitree-g1-mujoco --network host \
  -e DISPLAY="${DISPLAY:-:0}" -e QT_X11_NO_MITSHM=1 \
  -e LIBGL_ALWAYS_SOFTWARE=1 -e MUJOCO_GL=glfw \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  unitree-g1-mujoco:jazzy
