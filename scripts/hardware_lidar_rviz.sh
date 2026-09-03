#!/usr/bin/env bash
# Display the physical G1 Mid-360 cloud on the laptop.
# This container has host networking for DDS discovery only. It does not send
# motion commands and does not contain or launch the simulation stack.

set -euo pipefail

if [[ -z "${DISPLAY:-}" ]]; then
  echo "DISPLAY is unset. Run this from the laptop's graphical desktop session." >&2
  exit 1
fi
if ! command -v xhost >/dev/null; then
  echo "xhost is required for Docker to open an X11 RViz window." >&2
  exit 1
fi

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "${script_dir}/.." && pwd)"
rviz_config="${repo_root}/src/g1_bridge/rviz/g1_hardware_lidar.rviz"

if [[ ! -f "$rviz_config" ]]; then
  echo "RViz profile not found: $rviz_config" >&2
  exit 1
fi

# Allow only root from this local machine while the container is running.
xhost +si:localuser:root >/dev/null
cleanup() {
  xhost -si:localuser:root >/dev/null 2>&1 || true
}
trap cleanup EXIT

docker_args=(
  --rm
  --network host
  --ipc host
  -e "DISPLAY=${DISPLAY}"
  -e QT_X11_NO_MITSHM=1
  -e XDG_RUNTIME_DIR=/tmp/runtime-root
  -e "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}"
  -e ROS_LOCALHOST_ONLY=0
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw
  -v "${rviz_config}:/tmp/g1_hardware_lidar.rviz:ro"
)

# Software rendering is the stable default for this laptop's Docker/X11 path.
# Set RVIZ_GL=hardware to use /dev/dri instead.
case "${RVIZ_GL:-software}" in
  software)
    docker_args+=(-e LIBGL_ALWAYS_SOFTWARE=1 -e MESA_GL_VERSION_OVERRIDE=3.3 -e MESA_GLSL_VERSION_OVERRIDE=330)
    ;;
  hardware)
    docker_args+=(--device /dev/dri:/dev/dri)
    ;;
  *)
    echo "Unknown RVIZ_GL=${RVIZ_GL}. Use software or hardware." >&2
    exit 2
    ;;
esac

docker run "${docker_args[@]}" \
  osrf/ros:jazzy-desktop \
  rviz2 -d /tmp/g1_hardware_lidar.rviz "$@"
