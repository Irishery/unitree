#!/usr/bin/env bash
# Source this file: source scripts/use_network.sh <network-interface> [domain-id]

network_interface="${1:-}"
domain_id="${2:-0}"
workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"

if [[ -z "${network_interface}" ]]; then
  echo "Usage: source scripts/use_network.sh <network-interface> [domain-id]" >&2
  return 1 2>/dev/null || exit 1
fi

if ! ip link show dev "${network_interface}" >/dev/null 2>&1; then
  echo "Network interface '${network_interface}' does not exist" >&2
  return 1 2>/dev/null || exit 1
fi

if [[ -z "${ROS_DISTRO:-}" ]]; then
  if [[ -f /opt/ros/humble/setup.bash ]]; then
    source /opt/ros/humble/setup.bash
  elif [[ -f /opt/ros/jazzy/setup.bash ]]; then
    source /opt/ros/jazzy/setup.bash
  else
    echo "ROS 2 setup.bash was not found" >&2
    return 1 2>/dev/null || exit 1
  fi
fi

if [[ -f "${workspace_dir}/install/setup.bash" ]]; then
  source "${workspace_dir}/install/setup.bash"
fi

if [[ "${domain_id}" == "0" && "${ROS_DISTRO:-}" != "humble" ]]; then
  echo "Refusing ROS_DOMAIN_ID=0 with ROS_DISTRO=${ROS_DISTRO:-unset}: the physical G1 uses Humble." >&2
  echo "Use Humble for hardware, or a non-zero isolated domain for simulation." >&2
  return 1 2>/dev/null || exit 1
fi

export ROS_DOMAIN_ID="${domain_id}"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"${network_interface}\" priority=\"default\" multicast=\"default\" /></Interfaces></General></Domain></CycloneDDS>"

echo "ROS 2 network: interface=${network_interface}, domain=${ROS_DOMAIN_ID}, rmw=${RMW_IMPLEMENTATION}"
