#!/usr/bin/env bash
set -euo pipefail

exec docker exec -it unitree-g1-gazebo bash -lc \
  'source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash && exec ros2 run teleop_twist_keyboard teleop_twist_keyboard'

