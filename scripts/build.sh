#!/usr/bin/env bash
set -euo pipefail

workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
ros_distro="${ROS_DISTRO:-humble}"

if [[ ! -f "/opt/ros/${ros_distro}/setup.bash" ]]; then
  echo "ROS 2 ${ros_distro} is not installed in /opt/ros/${ros_distro}" >&2
  exit 1
fi

source "/opt/ros/${ros_distro}/setup.bash"
cd "${workspace_dir}"
colcon build --symlink-install --packages-up-to g1_bridge

