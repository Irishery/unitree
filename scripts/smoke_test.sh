#!/usr/bin/env bash
set -euo pipefail

echo "Waiting up to 10 seconds for the bridge..."
timeout 10 ros2 topic echo /g1/control_enabled --once
timeout 10 ros2 topic echo /g1/imu/data --once >/dev/null
timeout 10 ros2 topic echo /g1/joint_states --once >/dev/null

echo "Bridge telemetry is visible. Motion control state:"
ros2 topic echo /g1/control_enabled --once
echo "Do not enable motion until the G1 is supported and the area is clear."

