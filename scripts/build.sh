#!/usr/bin/env bash
set -euo pipefail

workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
ros_distro="${ROS_DISTRO:-humble}"

if [[ ! -f "/opt/ros/${ros_distro}/setup.bash" ]]; then
  echo "ROS 2 ${ros_distro} is not installed in /opt/ros/${ros_distro}" >&2
  exit 1
fi

# Some vendor ROS setup scripts dereference optional tracing variables without
# defaults.  Keep strict mode for this script, but do not impose `set -u` on
# the setup scripts we do not control.
set +u
source "/opt/ros/${ros_distro}/setup.bash"

# A stock G1 already carries the official Unitree message packages outside
# this workspace.  Source that overlay when present so g1_bridge can find
# unitree_hg and unitree_api during the build.  A fresh developer checkout
# that imports the packages into src/ continues to work without this file.
unitree_setup="${UNITREE_ROS_SETUP:-/home/unitree/unitree_ros2/install/setup.bash}"
if [[ -f "${unitree_setup}" ]]; then
  source "${unitree_setup}"
fi
set -u

cd "${workspace_dir}"
colcon build --symlink-install --packages-up-to g1_bridge
