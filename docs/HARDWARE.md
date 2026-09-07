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
SLAM, path planning, or any walking command.

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
export G1_HARDWARE_PEERS=10.0.88.165:7410
source scripts/hardware_env.sh wlxfc23cd952598 enP8p1s0
./scripts/hardware_mapping_safe.sh
```

Install the one missing dependency first if this launch reports that
`slam_toolbox` is unavailable:

```bash
sudo apt update
sudo apt install ros-humble-slam-toolbox
```

The safe wrapper starts the passive launch with `nice=10` and idle I/O priority
when supported. The physical profile processes every second 10 Hz cloud (5 Hz
scan), updates the map every 5 seconds and disables interactive graph editing.
It performs only this data path:

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
./scripts/hardware_mapping_safe.sh slam:=false
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

## Planning a path without movement

Do this only after `/map`, `/scan`, and the complete
`map -> odom -> base_footprint` TF chain remain healthy while the robot is in
Regular Mode. Install the Nav2 planning packages once on the robot:

```bash
sudo apt update
sudo apt install ros-humble-navigation2
```

Rebuild the updated workspace, keep telemetry, the separate Mid-360 launch and
mapping running, then start planning in another robot terminal:

```bash
cd /home/unitree/unitree
export G1_HARDWARE_PEERS=10.0.88.165:7410
source scripts/hardware_env.sh wlxfc23cd952598 enP8p1s0
ros2 launch g1_bridge hardware_planning.launch.py
```

This starts only `planner_server`, its global costmap, a lifecycle manager and
an RViz goal-to-planner adapter. It has no `controller_server`, `bt_navigator`,
velocity smoother, `/cmd_vel` publisher, or Unitree command publisher. Verify
that separation before requesting a path:

```bash
ros2 lifecycle get /planner_server
ros2 node list | grep -E 'planner|costmap|controller|bt_navigator|velocity'
ros2 topic info /cmd_vel
ros2 topic info /g1/motion_cmd_vel
timeout 8 ros2 topic hz /global_costmap/costmap
```

`planner_server` must be active; controller/BT/velocity nodes and velocity
publishers must be absent. On the laptop, restart the hardware RViz profile,
select **2D Goal Pose**, and click a known free cell. The green `/plan` line
must go around occupied and inflated cells. The same check can be requested by
coordinates without printing the entire path:

```bash
./scripts/hardware_plan_goal.py 1.0 0.0 0.0
```

Do not proceed if the path crosses the table, unknown map space, or comes
closer to an obstacle than the configured 0.45 m robot radius plus inflation.

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
upside-down. `odom` applies the calibrated robot TF and remains usable before
SLAM starts; Nav2 transforms an RViz goal from `odom` into `map`. If
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
unset G1_HARDWARE_PEERS
source scripts/hardware_env.sh enP8p1s0
ros2 launch g1_bridge hardware_motion.launch.py \
  motion_interface:=true allow_hardware_motion:=true
```

Stop `hardware_telemetry.launch.py` before this command because the motion
launch replaces it and supplies the same odometry/TF nodes. For the very first
bounded walking probe, also stop mapping and planning to isolate locomotion and
minimize CPU load; they can be reintroduced after the stop test passes. This
exposes the isolated `/g1/motion_cmd_vel` input and `/g1/enable_control`, but
still starts disarmed. It never publishes `/lowcmd`.

For the first test, keep the G1 on its gantry, clear its full fall radius, put
it in the normal Unitree Regular Mode, hold the official controller, and arm
the bridge explicitly:

```bash
ros2 service call /g1/enable_control std_srvs/srv/SetBool '{data: true}'
ros2 topic echo /g1/control_enabled --once
```

The second command must show `data: true`. Run the default fixed probe; it accepts no
arbitrary speed or duration arguments, sends only +0.05 m/s for 0.5 seconds, then sends
zero for 0.5 seconds and disables `/g1/enable_control`:

```bash
G1_ALLOW_MOTION_TEST=YES ./scripts/hardware_motion_probe.py
ros2 topic echo /g1/control_enabled --once
```

The final state must be `false`, and the robot must stop promptly. Disable at
once with the physical controller if it behaves unexpectedly. This probe is
the only motion stage added here: the Nav2 path is **not yet connected** to the
legs. Connecting a controller to `/g1/motion_cmd_vel` is permitted only after
the path clearance, bounded motion, watchdog stop, and manual disarm tests all
pass on the exact robot.

### Follow-up: nominal 20 cm walking probe

Only after the short probe has visibly moved the robot and software disarming
has been confirmed, the fixed `--twenty-cm` profile sends **0.05 m/s for 4 seconds**,
then zero for 0.5 seconds and requests disarming. The default remains 0.5 seconds.
This is a **timed** probe: 20 cm is nominal (`speed * time`), not a measured
distance or guaranteed minimum. It does not extend motion to compensate for
slippage, slow acceleration, or obstacles. The bridge uses the official
`LocoClient::Move()` bounded-command duration of 1 second; it refreshes that
command while ROS input is current. The independent 0.25-second ROS command
watchdog sends an explicit zero request when input stops. No native mode/FSM
switch is performed.

Keep the feet supporting the robot, the gantry as a safety restraint, the path
and fall radius clear, Regular Mode selected, and the official controller in
hand. This probe has no obstacle avoidance. With the updated motion bridge
already running, use a second **robot** terminal:

```bash
cd ~/unitree
unset G1_HARDWARE_PEERS
source scripts/hardware_env.sh enP8p1s0
ros2 service call /g1/enable_control std_srvs/srv/SetBool '{data: true}'
# Continue only on service success and data: true:
ros2 topic echo /g1/control_enabled --once
set -o pipefail
G1_ALLOW_MOTION_TEST=YES ./scripts/hardware_motion_probe.py --twenty-cm \
  2>&1 | tee ~/g1_motion_20cm.log
echo "probe_exit_code=${PIPESTATUS[0]}"
ros2 topic echo /g1/control_enabled --once
```

The final state must be false. Ctrl-C attempts stop/disarm; use the official
controller immediately if motion is unexpected or disarming is not confirmed.
Do not repeatedly run it to force a minimum distance.

### Diagnose a bounded probe that did not move

If the native API acknowledges nonzero commands but the robot only walks from
its official controller, run the **read-only locomotion check** before changing
speed or control modes. Keep the motion/telemetry bridge running **disarmed**,
the official controller sticks neutral, and do not run a walking probe alongside
this check:

```bash
cd ~/unitree
unset G1_HARDWARE_PEERS
source scripts/hardware_env.sh enP8p1s0
ros2 topic echo /g1/control_enabled --once
# Continue only if false:
python3 scripts/hardware_loco_check.py
```

It saves one `g1_hardware_logs/loco_check_*.log` file. It inspects local SDK
sources/metadata without importing the SDK; sends only API **7001, 7002, 7003**
(GetFsmId, GetFsmMode, GetBalanceMode), once each, with a 5-second response timeout;
and observes other `/api/sport/request` traffic for another 10 seconds. There is
no velocity command (including zero), arming service call, retry, native mode
switch, or firmware/library update. Missing/true `control_enabled` aborts the
check; it does not silently disarm the robot for the user.

The current upstream Python SDK exposes `SwitchToUserCtrl`/`SwitchToInternalCtrl`,
but their presence does **not** prove they are required or safe for this firmware.
This check never calls them. SDK source versions are not firmware versions.
Raw FSM values must be interpreted against the robot's actual SDK/firmware,
not inferred from the app's Regular Mode label.

Observation is passive and BEST_EFFORT; zero observed competing messages is not
proof that arbitration cannot occur during movement. Humble's Python message
metadata cannot tie each request to a publisher GID; the endpoint inventory
is logged separately and does not establish which publisher sent a request.
Do not disable native publishers or the official controller based on their count.

Do not increase velocity/duration or switch native FSM modes to guess the cause.
The bridge now subscribes to `/api/sport/response` **only in motion-interface
mode**. It matches both our request ID and API 7105, and records response status
codes. Foreign, duplicate and expired replies are ignored. The diagnostic
timeout is 5 seconds (`command_response_timeout_s`); it does not retry commands,
re-arm the bridge, or alter the existing command/low-state watchdogs.

Capture a fresh launch log on the robot (stop the previous project motion launch
first, do not run two bridges):

```bash
cd ~/unitree
unset G1_HARDWARE_PEERS
source scripts/hardware_env.sh enP8p1s0
set -o pipefail
ros2 launch g1_bridge hardware_motion.launch.py \
  motion_interface:=true allow_hardware_motion:=true \
  2>&1 | tee ~/g1_motion_start.log
```

This still starts **disarmed**. Only after the physical checklist above, use
another robot terminal to explicitly arm and run the same fixed probe:

```bash
cd ~/unitree
unset G1_HARDWARE_PEERS
source scripts/hardware_env.sh enP8p1s0
ros2 service call /g1/enable_control std_srvs/srv/SetBool '{data: true}'
# Continue only if the service succeeds and this shows true:
ros2 topic echo /g1/control_enabled --once
set -o pipefail
G1_ALLOW_MOTION_TEST=YES ./scripts/hardware_motion_probe.py \
  2>&1 | tee ~/g1_motion_probe.log
echo "probe_exit_code=${PIPESTATUS[0]}"
```

The probe now reports successful **software disarming** only after a successful
SetBool response and a fresh `control_enabled=false` message. Exit code 3 means
disarming was not confirmed: use the official controller and do not repeat the
probe. Ctrl-C/SIGTERM attempts bounded cleanup while ROS is still alive; SIGKILL,
process/transport failure cannot guarantee a stop. Physical observation and the
official controller remain necessary. Exit code 0 is not proof of walking.

Keep the bridge running disarmed for at least 6 seconds after the probe so all
pending replies can expire, then collect (read-only):

```bash
timeout 5 ros2 topic echo /diagnostics diagnostic_msgs/msg/DiagnosticArray \
  --once > ~/g1_motion_diagnostics.log
```

Send all three logs. Counters are cumulative since bridge startup, **not reset
on disarm**:

| Evidence | Meaning |
| --- | --- |
| `cmd_nonzero_count=0` | No valid nonzero input observed by the bridge |
| `cmd_nonzero_count>0`, `nonzero_requests_sent=0` | Input arrived but no nonzero request was published; inspect arming/watchdog logs |
| `nonzero_response_timeouts>0` | No matching reply within the deadline; does not prove acceptance or rejection |
| `nonzero_responses_rejected>0` | Native API returned a nonzero status; inspect the logged code and request ID |
| `nonzero_responses_accepted>0` | Native API replied with status 0; actual walking still requires independent confirmation |

`responses_accepted` includes **zero** requests; use `nonzero_responses_accepted`
when diagnosing walking. `last_response_code=0` is meaningful only together
with `responses_received>0`. Historical timeouts/rejections keep the command
diagnostic at WARN until node restart. No low-level motor command publisher,
native mode changes, or automatic retry was added.

The response fields and status interpretation follow the official
[Unitree BaseClient](https://github.com/unitreerobotics/unitree_ros2/blob/master/example/src/include/common/base_client.hpp).
Unlike its blocking call, the bridge observes replies asynchronously so reply
waiting cannot block the existing command watchdog.

Local regression checks (no connection to the robot):

```bash
python3 -m unittest discover -s scripts/tests -p test_hardware_motion_probe.py -v
colcon test --packages-select g1_bridge
colcon test-result --verbose
```
