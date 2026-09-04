#!/usr/bin/env bash
# Start passive physical-G1 mapping below the priority of vendor control tasks.
# This script contains no motion command and assumes hardware_env.sh was sourced.

set -euo pipefail

if [[ "${ROS_DISTRO:-}" != "humble" || \
      "${RMW_IMPLEMENTATION:-}" != "rmw_cyclonedds_cpp" || \
      "${ROS_DOMAIN_ID:-}" != "0" ]]; then
  echo "Source scripts/hardware_env.sh before starting physical mapping." >&2
  exit 2
fi

mapping_nice="${G1_MAPPING_NICE:-10}"
if [[ ! "${mapping_nice}" =~ ^([0-9]|1[0-9])$ ]]; then
  echo "G1_MAPPING_NICE must be an integer from 0 to 19." >&2
  exit 2
fi

launcher=(nice -n "${mapping_nice}")
if command -v ionice >/dev/null 2>&1; then
  launcher+=(ionice -c 3)
fi

echo "Starting passive mapping with nice=${mapping_nice}, I/O class=idle when available."
exec "${launcher[@]}" ros2 launch g1_bridge hardware_mapping.launch.py "$@"
