# Physical G1 bringup

This procedure is for the confirmed G1 EDU 29-DoF computer running Ubuntu
22.04 and ROS 2 Humble. Every process connected to the physical DDS domain
must use Humble and CycloneDDS. The first launch uses a dedicated telemetry
executable that does not link `unitree_api`, create `/g1/enable_control`,
subscribe to `/cmd_vel`, or publish Unitree sport commands.

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

After the build, use the repository's guarded environment in every new robot
terminal. The confirmed robot uses `enP8p1s0` for its internal Unitree/Livox
network and `wlxfc23cd952598` for the `10.0.88.0/24` laptop network, so expose
CycloneDDS on both interfaces:

```bash
cd /home/unitree/unitree
export G1_HARDWARE_PEERS=10.0.88.165:7410
source scripts/hardware_env.sh wlxfc23cd952598 enP8p1s0
```

It fixes `ROS_DISTRO=humble`, `ROS_DOMAIN_ID=0` and
`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`, rejects a missing CycloneDDS RMW and
clears inherited Fast DDS profile variables. Supplying both interfaces lets
the bridge receive the native G1 topics on `192.168.123.0/24` and advertise
its ROS outputs on Wi-Fi to the laptop. Port `7410` is the fixed CycloneDDS
discovery port used by the single-process RViz container on domain 0; specifying
it avoids slow peer-port scanning over Wi-Fi. It deliberately does not add
`/usr/local/lib` globally to `LD_LIBRARY_PATH`.

If `robot_state_publisher` is missing:

```bash
sudo apt update
sudo apt install ros-humble-robot-state-publisher ros-humble-tf2-ros \
  ros-humble-rmw-cyclonedds-cpp
```

## First launch without movement

Terminal 1 on the robot:

```bash
cd /home/unitree/unitree
export G1_HARDWARE_PEERS=10.0.88.165:7410
source scripts/hardware_env.sh wlxfc23cd952598 enP8p1s0
ros2 launch g1_bridge hardware_telemetry.launch.py
```

`hardware_bringup.launch.py` and `bridge.launch.py` are retained as safe
aliases for this command. None of them accepts a motion enable argument and
none starts the DEX3 bridge.

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

## Head Livox Mid-360: raw cloud and reduced RViz relay

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
cd /home/unitree/unitree
export G1_HARDWARE_PEERS=10.0.88.165:7410
source scripts/hardware_env.sh wlxfc23cd952598 enP8p1s0
ros2 launch g1_bridge mid360.launch.py
```

The launch keeps the full `/mid360/points` cloud for robot-local processing and
publishes `/mid360/points_rviz` for the laptop. By default the relay takes every
second frame and every fourth point, reducing the network payload by about 8x
without modifying the raw topic. `/mid360/imu` is unchanged. All three messages
use the URDF frame `mid360_link`. Verify the data and frame without moving the
robot:

```bash
timeout 8 ros2 topic hz /mid360/points
timeout 8 ros2 topic hz /mid360/points_rviz
ros2 topic echo /mid360/points --once
ros2 run tf2_ros tf2_echo pelvis mid360_link
```

The relay can be tuned at launch time, for example
`rviz_frame_stride:=2 rviz_point_stride:=8`, or disabled with
`rviz_relay:=false`. These options affect only the laptop visualization topic.

## Passive 2-D SLAM from the real Mid-360

After the TF checks pass, start the following in a **third** robot terminal:

```bash
cd /home/unitree/unitree
source scripts/hardware_env.sh wlxfc23cd952598 enP8p1s0
ros2 launch g1_bridge hardware_mapping.launch.py
```

Install the one missing dependency first if this launch reports that
`slam_toolbox` is unavailable:

```bash
sudo apt update
sudo apt install ros-humble-slam-toolbox
```

The launch performs only this data path:

```text
/mid360/points -> g1_mid360_scan_projector -> /scan -> SLAM Toolbox -> /map
```

The projector transforms each cloud point to `base_footprint`, rejects the
ground (`z < 0.12 m`), high ceiling returns (`z > 1.60 m`) and the G1 body box
around the base, then publishes the nearest remaining point in every planar
beam. It has **no** `/cmd_vel` subscriber or publisher, no Nav2 process, no
`/g1/enable_control` service and no `/api/sport/request` publisher.

First inspect the projection without SLAM:

```bash
ros2 launch g1_bridge hardware_mapping.launch.py slam:=false
timeout 8 ros2 topic hz /scan
```

Then restart it with the default `slam:=true`. The existing laptop RViz profile
now includes the cyan filtered `/scan` and the saved online `/map`; keep
`Fixed Frame = odom`. The map will update only when the robot's **native**
odometry changes. During this validation the robot must be moved only through
the official Unitree interface under supervision; this repository still does
not command walking.

Save a completed map on the robot with:

```bash
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
  "{name: {data: '/home/unitree/g1_maps/room'}}"
```

Create `/home/unitree/g1_maps` first if needed. Nav2 is intentionally not part
of this launch; it will be added only after the scan and map have been reviewed.

### RViz on the Ubuntu 24.04 laptop

The laptop has Ubuntu 24.04 without a native ROS installation. Build the
dedicated ROS 2 Humble + CycloneDDS viewer once; do not attach a Jazzy
participant to the physical Humble domain.

```bash
cd /home/kir/unitree
./scripts/hardware_rviz_build.sh
```

The viewer uses host networking solely to receive DDS topics from the robot;
it does not start MuJoCo, SLAM, Nav2, or any motion publisher.

Keep `hardware_telemetry.launch.py` and `mid360.launch.py` running on the robot.
On the laptop graphical desktop, connected to the same DDS-capable network,
run:

```bash
cd /home/kir/unitree
./scripts/hardware_lidar_rviz.sh
```

If the laptop has several active interfaces, select the one connected to the
G1 explicitly:

```bash
G1_HARDWARE_NETWORK_INTERFACE=enp3s0 ./scripts/hardware_lidar_rviz.sh
```

For the confirmed Wi-Fi addresses, bypass multicast-only discovery with an
explicit robot peer:

```bash
G1_HARDWARE_NETWORK_INTERFACE=wlp3s0 \
G1_HARDWARE_PEERS=10.0.88.180 \
  ./scripts/hardware_lidar_rviz.sh
```

The viewer reserves CycloneDDS participant index 0, hence UDP port `7410` on
domain 0. Run only one hardware RViz container at a time, and configure the
robot peer as `10.0.88.165:7410` as shown above.

The profile selects `odom` as the fixed frame and displays the reduced
`/mid360/points_rviz` topic. The full `/mid360/points` topic remains available
locally on the robot for projection, mapping and later navigation.
The factory URDF mounts `mid360_link` with an approximately 180-degree roll;
using the sensor frame as RViz's fixed frame therefore makes the raw view look
upside-down. `odom` applies that fixed TF and is the correct world view. If
the window opens but no points appear, first verify DDS visibility from the
same laptop terminal:

```bash
docker run --rm --network host -e ROS_DOMAIN_ID=0 -e ROS_LOCALHOST_ONLY=0 \
  -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  unitree-g1-hardware-rviz:humble \
  bash -lc 'source /opt/ros/humble/setup.bash && timeout 8 ros2 topic hz /mid360/points_rviz'
```

## Isolating DDS serialization faults

`sequence size exceeds remaining buffer` is a CDR deserialization error, not
an out-of-memory message. If it appears, stop the laptop viewer and all project
nodes. After a clean robot-only baseline test, the Mid-360 driver can be tested
without joining the native robot graph:

```bash
set +u
source /opt/ros/humble/setup.bash
source /home/unitree/unitree_ros2/install/setup.bash
source /home/unitree/unitree/install/setup.bash
source /home/unitree/livox_ws/install/setup.bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
unset FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE
ros2 launch g1_bridge mid360.launch.py
```

The LiDAR UDP connection still works in this test, while its ROS publications
cannot enter the robot's domain 0. If the CDR errors disappear, inspect foreign
domain-0 participants and verify that all of them use Humble/CycloneDDS before
continuing.

## Physical locomotion gate

Do not run this stage while investigating DDS or `Motor to PC Timeout` faults.
After the passive hardware checklist has passed, stop the telemetry launch and
start the separately gated high-level interface:

```bash
cd /home/unitree/unitree
source scripts/hardware_env.sh wlxfc23cd952598 enP8p1s0
ros2 launch g1_bridge hardware_motion.launch.py \
  motion_interface:=true allow_hardware_motion:=true
```

This exposes `/cmd_vel` and `/g1/enable_control` but still starts disarmed. It
never publishes `/lowcmd`. The service may be enabled only with the robot on
its gantry, a clear fall radius, the official controller in hand, fresh
`/lowstate`, and no other sport-request publisher under test.
