#!/usr/bin/env bash
set -euo pipefail

xhost +si:localuser:root >/dev/null
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# Do not inherit ROS_DOMAIN_ID, CycloneDDS or Fast DDS settings from the host:
# they may be configured for the real robot.
source "${script_dir}/mujoco_env.sh"
mujoco_domain_id="$(mujoco_ros_domain_id)"
# MuJoCo's GLFW viewer uses the X11 MIT-SHM extension.  It needs the host IPC
# namespace so the X server can attach the viewer's shared-memory buffer.
docker run --rm --name unitree-g1-mujoco --network host --ipc=host \
  -e "ROS_DOMAIN_ID=${mujoco_domain_id}" -e ROS_LOCALHOST_ONLY=1 \
  -e DISPLAY="${DISPLAY:-:0}" -e QT_X11_NO_MITSHM=1 \
  -e XDG_RUNTIME_DIR=/tmp/runtime-root \
  -e LIBGL_ALWAYS_SOFTWARE=1 -e MESA_GL_VERSION_OVERRIDE=3.3 \
  -e MESA_GLSL_VERSION_OVERRIDE=330 -e MUJOCO_GL=glfw \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  unitree-g1-mujoco:jazzy \
  ros2 launch g1_mujoco sim.launch.py "$@"
