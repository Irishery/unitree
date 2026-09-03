#!/usr/bin/env bash
# Shared safety boundary for all local MuJoCo tools.
#
# A physical G1 normally uses ROS_DOMAIN_ID=0.  The simulation must never join
# that DDS domain: host-network Docker otherwise makes its ROS graph visible to
# the robot.  Keep MuJoCo on loopback and in a non-zero domain instead.

mujoco_ros_domain_id() {
  local domain_id="${MUJOCO_ROS_DOMAIN_ID:-42}"

  if ! [[ "${domain_id}" =~ ^[0-9]+$ ]] || (( domain_id > 232 )); then
    echo "MUJOCO_ROS_DOMAIN_ID must be an integer from 1 through 232 (got '${domain_id}')." >&2
    return 2
  fi
  if (( domain_id == 0 )); then
    echo "Refusing ROS_DOMAIN_ID=0 for MuJoCo: domain 0 is reserved for the physical G1." >&2
    echo "Use the default MUJOCO_ROS_DOMAIN_ID=42, or choose another non-zero domain." >&2
    return 2
  fi

  printf '%s\n' "${domain_id}"
}
