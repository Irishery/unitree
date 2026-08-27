#!/usr/bin/env bash
# Install the official Livox SDK2 and ROS 2 driver on the G1 computer.
#
# This script only installs and builds the LiDAR driver.  It never starts
# locomotion, publishes /cmd_vel, or calls Unitree APIs.

set -euo pipefail

LIVOX_WORKSPACE="${LIVOX_WORKSPACE:-/home/unitree/livox_ws}"
SDK_TAG="${LIVOX_SDK_TAG:-v1.3.1}"
DRIVER_TAG="${LIVOX_DRIVER_TAG:-1.2.6}"
ROS_SETUP="/opt/ros/humble/setup.bash"

if [[ ! -f "$ROS_SETUP" ]]; then
  echo "ROS 2 Humble was not found at $ROS_SETUP" >&2
  exit 1
fi

echo "Installing build dependencies (sudo is required)."
sudo apt update
sudo apt install -y \
  build-essential cmake git libapr1-dev libboost-system-dev libboost-thread-dev \
  libpcl-dev ros-humble-ament-cmake-auto ros-humble-pcl-conversions \
  ros-humble-pcl-ros ros-humble-rosidl-default-generators ros-humble-tf2-ros

mkdir -p "$LIVOX_WORKSPACE/src"

if [[ ! -d "$LIVOX_WORKSPACE/src/Livox-SDK2/.git" ]]; then
  git clone --depth 1 --branch "$SDK_TAG" \
    https://github.com/Livox-SDK/Livox-SDK2.git "$LIVOX_WORKSPACE/src/Livox-SDK2"
fi
if [[ ! -d "$LIVOX_WORKSPACE/src/livox_ros_driver2/.git" ]]; then
  git clone --depth 1 --branch "$DRIVER_TAG" \
    https://github.com/Livox-SDK/livox_ros_driver2.git \
    "$LIVOX_WORKSPACE/src/livox_ros_driver2"
fi

cmake -S "$LIVOX_WORKSPACE/src/Livox-SDK2" \
  -B "$LIVOX_WORKSPACE/build/livox_sdk" \
  -DCMAKE_BUILD_TYPE=Release
cmake --build "$LIVOX_WORKSPACE/build/livox_sdk" --parallel 2
sudo cmake --install "$LIVOX_WORKSPACE/build/livox_sdk"
sudo ldconfig

# The ROS setup scripts use variables that may be unset in strict bash mode.
set +u
source "$ROS_SETUP"
set -u
cd "$LIVOX_WORKSPACE"
colcon build --symlink-install --packages-select livox_ros_driver2

echo
echo "Installed successfully. Start the driver with:"
echo "  source /opt/ros/humble/setup.bash"
echo "  source /home/unitree/unitree_ros2/install/setup.bash"
echo "  source /home/unitree/unitree/install/setup.bash"
echo "  source $LIVOX_WORKSPACE/install/setup.bash"
echo "  export LD_LIBRARY_PATH=/usr/local/lib:\${LD_LIBRARY_PATH:-}"
echo "  ros2 launch g1_bridge mid360.launch.py"
