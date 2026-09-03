#!/usr/bin/env bash
set -eo pipefail

workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
run_id="$(date +%Y%m%d_%H%M%S)"
log_dir="${G1_HARDWARE_LOG_DIR:-${workspace_dir}/g1_hardware_logs}"
log_file="${log_dir}/telemetry_${run_id}.log"
mkdir -p "${log_dir}"

exec 3>&1
echo "Writing hardware telemetry log to: ${log_file}" >&3
exec >"${log_file}" 2>&1

set +u
if [[ -n "${G1_HARDWARE_NETWORK_INTERFACES:-}" ]]; then
  read -r -a hardware_network_interfaces <<<"${G1_HARDWARE_NETWORK_INTERFACES}"
  source "${workspace_dir}/scripts/hardware_env.sh" "${hardware_network_interfaces[@]}"
elif [[ -n "${G1_HARDWARE_NETWORK_INTERFACE:-}" ]]; then
  source "${workspace_dir}/scripts/hardware_env.sh" "${G1_HARDWARE_NETWORK_INTERFACE}"
else
  source "${workspace_dir}/scripts/hardware_env.sh"
fi
set -u

run() {
  echo
  echo "$ $*"
  "$@" || true
}

run_timeout() {
  local seconds="$1"
  shift
  echo
  echo "$ timeout ${seconds} $*"
  timeout "${seconds}" "$@" || true
}

echo "Unitree G1 telemetry-only bringup check"
date --iso-8601=seconds
echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
echo "ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY}"
echo "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}"

run ros2 node list
run ros2 topic list
run ros2 service list
run ros2 topic info -v /g1/joint_states
run ros2 topic info -v /g1/imu/data
run ros2 topic info -v /odom
run ros2 topic info -v /tf
run_timeout 5 ros2 topic hz /g1/joint_states
run_timeout 5 ros2 topic hz /g1/imu/data
run_timeout 5 ros2 topic hz /odom
run_timeout 3 ros2 topic echo /g1/control_enabled --once
run_timeout 3 ros2 topic echo /g1/joint_states --once
run_timeout 3 ros2 topic echo /g1/imu/data --once
run_timeout 3 ros2 topic echo /odom --once
run_timeout 4 ros2 run tf2_ros tf2_echo odom base_footprint
run_timeout 4 ros2 run tf2_ros tf2_echo base_footprint pelvis
run_timeout 4 ros2 run tf2_ros tf2_echo pelvis torso_link

echo
result=0
for required_topic in /g1/joint_states /g1/imu/data /g1/control_enabled /odom /tf /tf_static; do
  if ros2 topic list | grep -qx "${required_topic}"; then
    echo "TOPIC CHECK OK: ${required_topic}"
  else
    echo "TOPIC CHECK FAILED: ${required_topic} is absent."
    result=1
  fi
done

if ros2 service list | grep -qx /g1/enable_control; then
  echo "SAFETY CHECK FAILED: /g1/enable_control exists; telemetry-only mode is not active."
  result=1
else
  echo "SAFETY CHECK OK: /g1/enable_control is absent."
fi

echo "Log complete: ${log_file}"
echo "Hardware telemetry check finished. Send this file for review:" >&3
echo "${log_file}" >&3
exit "${result}"
