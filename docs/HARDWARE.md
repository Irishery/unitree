# Physical G1 bringup

This procedure is for the confirmed G1 EDU 29-DoF computer running Ubuntu
22.04 and ROS 2 Humble. The first launch is deliberately telemetry-only: it
does not create `/g1/enable_control`, does not subscribe to `/cmd_vel`, and
does not publish Unitree sport commands.

## Confirmed native sources

The robot measurements from 2026-08-27 established these inputs:

| Native topic | Rate | Use |
|---|---:|---|
| `/lowstate` | about 1 kHz | joint state and torso IMU source |
| `/state_estimator/odom_pelvis` | about 52 Hz | measured planar odometry source |

The native odometry reports `frame_id=odom` and `child_frame_id=pelvis`, but
its observed `z` value was about 0.21 m. That value is not used as the URDF
pelvis height. `g1_odom_tf` extracts measured `x`, `y`, yaw, roll and pitch,
then publishes this navigation-friendly chain:

```text
odom -> base_footprint -> pelvis -> URDF links
```

`base_footprint -> pelvis` defaults to the model's nominal 0.793 m height.

No publisher was present on `/unitree/slam_mapping/points`,
`/unitree/slam_relocation/points` or `/utlidar/range_info`. Consequently this
bringup does not start SLAM or Nav2 yet. Enabling Nav2 without a real obstacle
cloud would make collision avoidance unsafe.

## Build on the robot

Transfer the updated repository to `/home/unitree/unitree`, then run:

```bash
cd /home/unitree/unitree
source /home/unitree/g1_ros_env.sh
./scripts/build.sh
source install/setup.bash
```

If `robot_state_publisher` is missing:

```bash
sudo apt update
sudo apt install ros-humble-robot-state-publisher ros-humble-tf2-ros
```

## First launch without movement

Terminal 1 on the robot:

```bash
cd /home/unitree/unitree
source /home/unitree/g1_ros_env.sh
source install/setup.bash
ros2 launch g1_bridge hardware_bringup.launch.py
```

Do not pass `motion_interface:=true` during this stage. The default launch also
keeps the DEX3 bridge off.

Terminal 2 on the robot:

```bash
cd /home/unitree/unitree
./scripts/hardware_telemetry_check.sh
```

The script writes the complete output to `g1_hardware_logs/` and prints only
the log path in the terminal. A successful safety line says that
`/g1/enable_control` is absent.

## Expected interfaces

After launch:

- `/g1/joint_states` and `/g1/imu/data` should publish near 50 Hz;
- `/odom` should publish near 50 Hz with child frame `base_footprint`;
- TF lookup must work for `odom -> base_footprint -> pelvis -> torso_link`;
- `/g1/control_enabled` must be `false`;
- `/g1/enable_control` must not exist in telemetry-only mode.

The next stage is to expose this standard ROS graph to the laptop and inspect
the physical model in RViz. LiDAR activation and frame calibration come before
SLAM, Nav2, or any walking command.

## Head Livox Mid-360: raw cloud only

The physical Mid-360 was reachable on the robot internal network at
`192.168.123.120`. Its existing G1Pilot sample configuration was not usable on
this robot: it sent UDP data to `192.168.123.123`, while the robot computer is
`192.168.123.164`. The repository now contains a corrected driver configuration
in `g1_mid360_192_168_123_164.json`.

This is deliberately a separate launch. It does not start SLAM, Nav2, the
motion interface, DEX3, or any Unitree command publisher. Do **not** launch the
whole G1Pilot stack for this test.

Install the official Livox SDK2 and ROS 2 driver once on the robot:

```bash
cd /home/unitree/unitree
./scripts/install_livox_mid360_driver.sh
```

Then keep the telemetry-only bringup from the previous section running and, in
another terminal, start only the LiDAR:

```bash
source /opt/ros/humble/setup.bash
source /home/unitree/unitree_ros2/install/setup.bash
source /home/unitree/unitree/install/setup.bash
source /home/unitree/livox_ws/install/setup.bash
export LD_LIBRARY_PATH=/usr/local/lib:${LD_LIBRARY_PATH:-}
ros2 launch g1_bridge mid360.launch.py
```

The expected topics are `/mid360/points` (`sensor_msgs/PointCloud2`) and
`/mid360/imu`, both in the URDF frame `mid360_link`. Verify the data and frame
without moving the robot:

```bash
timeout 8 ros2 topic hz /mid360/points
ros2 topic echo /mid360/points --once
ros2 run tf2_ros tf2_echo pelvis mid360_link
```

Only after those checks and an RViz visual inspection of ground orientation,
sensor direction, and the point cloud placement should we create a filtered
navigation scan/costmap input. The raw 3-D cloud must not be fed directly into
the existing 2-D navigation configuration.
