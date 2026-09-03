#!/usr/bin/env bash
# Source this file on the physical G1 before starting repository hardware nodes:
#   source scripts/hardware_env.sh [network-interface ...]

_g1_hardware_had_nounset=false
case "$-" in
  *u*)
    _g1_hardware_had_nounset=true
    set +u
    ;;
esac

_g1_hardware_setup() {
  local script_dir workspace_dir unitree_setup network_interface interfaces_xml interface_list
  local -a network_interfaces=()

  script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
  workspace_dir="$(cd -- "${script_dir}/.." && pwd)"
  unitree_setup="${UNITREE_ROS_SETUP:-/home/unitree/unitree_ros2/install/setup.bash}"

  if [[ $# -gt 0 ]]; then
    for network_interface in "$@"; do
      [[ -n "${network_interface}" ]] && network_interfaces+=("${network_interface}")
    done
  elif [[ -n "${G1_HARDWARE_NETWORK_INTERFACES:-}" ]]; then
    read -r -a network_interfaces <<<"${G1_HARDWARE_NETWORK_INTERFACES}"
  elif [[ -n "${G1_HARDWARE_NETWORK_INTERFACE:-}" ]]; then
    network_interfaces+=("${G1_HARDWARE_NETWORK_INTERFACE}")
  fi

  if [[ ! -f /opt/ros/humble/setup.bash ]]; then
    echo "G1 hardware environment error: /opt/ros/humble/setup.bash is missing" >&2
    return 1
  fi
  source /opt/ros/humble/setup.bash

  if [[ ! -f "${unitree_setup}" ]]; then
    echo "G1 hardware environment error: Unitree ROS overlay is missing: ${unitree_setup}" >&2
    return 1
  fi
  source "${unitree_setup}"

  if [[ ! -f "${workspace_dir}/install/setup.bash" ]]; then
    echo "G1 hardware environment error: project overlay is missing; build ${workspace_dir} first" >&2
    return 1
  fi
  source "${workspace_dir}/install/setup.bash"

  if [[ -f /home/unitree/livox_ws/install/setup.bash ]]; then
    source /home/unitree/livox_ws/install/setup.bash
  fi

  export ROS_DOMAIN_ID=0
  export ROS_LOCALHOST_ONLY=0
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  unset FASTRTPS_DEFAULT_PROFILES_FILE
  unset FASTDDS_DEFAULT_PROFILES_FILE

  if [[ ${#network_interfaces[@]} -gt 0 ]]; then
    interfaces_xml=""
    interface_list=""
    for network_interface in "${network_interfaces[@]}"; do
      if [[ ! "${network_interface}" =~ ^[[:alnum:]_.:-]+$ ]]; then
        echo "G1 hardware environment error: invalid network interface name: ${network_interface}" >&2
        return 1
      fi
      if ! ip link show dev "${network_interface}" >/dev/null 2>&1; then
        echo "G1 hardware environment error: network interface does not exist: ${network_interface}" >&2
        return 1
      fi
      interfaces_xml+="<NetworkInterface name=\"${network_interface}\" priority=\"default\" multicast=\"default\" />"
      interface_list+="${interface_list:+,}${network_interface}"
    done
    export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces>${interfaces_xml}</Interfaces></General></Domain></CycloneDDS>"
  else
    interface_list="auto"
    unset CYCLONEDDS_URI
  fi

  if [[ "${ROS_DISTRO:-}" != "humble" ]]; then
    echo "G1 hardware environment error: expected ROS_DISTRO=humble, got ${ROS_DISTRO:-unset}" >&2
    return 1
  fi
  if ! command -v ros2 >/dev/null 2>&1; then
    echo "G1 hardware environment error: ros2 is unavailable" >&2
    return 1
  fi
  if ! ros2 pkg prefix rmw_cyclonedds_cpp >/dev/null 2>&1; then
    echo "G1 hardware environment error: rmw_cyclonedds_cpp is not installed" >&2
    return 1
  fi

  echo "G1 hardware ROS environment: distro=${ROS_DISTRO}, domain=${ROS_DOMAIN_ID}, rmw=${RMW_IMPLEMENTATION}, interfaces=${interface_list}"
}

if _g1_hardware_setup "$@"; then
  _g1_hardware_status=0
else
  _g1_hardware_status=$?
fi
unset -f _g1_hardware_setup
if [[ "${_g1_hardware_had_nounset}" == true ]]; then
  set -u
fi
unset _g1_hardware_had_nounset
if [[ ${_g1_hardware_status} -ne 0 ]]; then
  return "${_g1_hardware_status}" 2>/dev/null || exit "${_g1_hardware_status}"
fi
unset _g1_hardware_status
