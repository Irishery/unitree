#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 ]]; then
  echo "Usage: $0 models/checkpoints/g1_pick_sac_200000_steps.zip [evaluate_rl options]" >&2
  exit 2
fi

xhost +si:localuser:root >/dev/null
docker run --rm --network host --ipc=host \
  -e DISPLAY="${DISPLAY:-:0}" -e QT_X11_NO_MITSHM=1 \
  -e LIBGL_ALWAYS_SOFTWARE=1 -e MUJOCO_GL=glfw \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v "$PWD/models:/ws/models:ro" \
  unitree-g1-mujoco:jazzy \
  ros2 run g1_mujoco evaluate_rl --model "/ws/$1" "${@:2}"
