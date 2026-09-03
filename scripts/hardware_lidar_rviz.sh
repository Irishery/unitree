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
image_name="unitree-g1-hardware-rviz:humble"

if [[ ! -f "$rviz_config" ]]; then
  echo "RViz profile not found: $rviz_config" >&2
  exit 1
fi

hardware_domain_id="${ROS_DOMAIN_ID:-0}"
hardware_network_interface="${G1_HARDWARE_NETWORK_INTERFACE:-}"
hardware_peers="${G1_HARDWARE_PEERS:-}"
declare -a cyclonedds_peer_array=()
if [[ "${hardware_domain_id}" != "0" ]]; then
  echo "Physical G1 RViz requires ROS_DOMAIN_ID=0 (got ${hardware_domain_id})." >&2
  exit 2
fi
if [[ -n "${hardware_network_interface}" ]]; then
  if ! ip link show dev "${hardware_network_interface}" >/dev/null 2>&1; then
    echo "G1 hardware network interface does not exist: ${hardware_network_interface}" >&2
    exit 2
  fi
fi
if ! docker image inspect "${image_name}" >/dev/null 2>&1; then
  echo "Hardware RViz image is missing: ${image_name}" >&2
  echo "Build it first with: ./scripts/hardware_rviz_build.sh" >&2
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
  -e "ROS_DOMAIN_ID=${hardware_domain_id}"
  -e ROS_LOCALHOST_ONLY=0
  -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw
  -v "${rviz_config}:/tmp/g1_hardware_lidar.rviz:ro"
)

cyclonedds_general=""
cyclonedds_discovery=""
if [[ -n "${hardware_network_interface}" ]]; then
  cyclonedds_general="<General><Interfaces><NetworkInterface name=\"${hardware_network_interface}\" priority=\"default\" multicast=\"default\" /></Interfaces></General>"
fi
if [[ -n "${hardware_peers}" ]]; then
  cyclonedds_peers=""
  read -r -a cyclonedds_peer_array <<<"${hardware_peers}"
  for peer in "${cyclonedds_peer_array[@]}"; do
    if [[ ! "${peer}" =~ ^[[:alnum:]_.:-]+$ ]]; then
      echo "Invalid CycloneDDS peer: ${peer}" >&2
      exit 2
    fi
    cyclonedds_peers+="<Peer Address=\"${peer}\" />"
  done
  cyclonedds_discovery="<Discovery><Peers>${cyclonedds_peers}</Peers></Discovery>"
fi
if [[ -n "${cyclonedds_general}${cyclonedds_discovery}" ]]; then
  docker_args+=(
    -e "CYCLONEDDS_URI=<CycloneDDS><Domain>${cyclonedds_general}${cyclonedds_discovery}</Domain></CycloneDDS>"
  )
fi

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
  "${image_name}" \
  rviz2 -d /tmp/g1_hardware_lidar.rviz "$@"
