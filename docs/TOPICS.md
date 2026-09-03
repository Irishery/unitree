# ROS 2 contract

## Input to the bridge

| Name | Type | Meaning |
|---|---|---|
| `lowstate` | `unitree_hg/msg/LowState` | Native G1 joint and torso IMU state |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Requested walking velocity in the robot body frame |
| `/g1/enable_control` | `std_srvs/srv/SetBool` | Explicitly arm or disarm motion commands |

Only `linear.x`, `linear.y`, and `angular.z` are used from `/cmd_vel`. Each value is clamped by the configured limit.

## Output from the bridge

| Name | Type | Meaning |
|---|---|---|
| `/g1/joint_states` | `sensor_msgs/msg/JointState` | G1 position, velocity, and estimated effort |
| `/g1/imu/data` | `sensor_msgs/msg/Imu` | Torso orientation, gyro, and acceleration |
| `/g1/control_enabled` | `std_msgs/msg/Bool` | Latched motion-control state |
| `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | Low-state and watchdog health |
| `/api/sport/request` | `unitree_api/msg/Request` | Native G1 locomotion request (API `7105`) |
| `/odom` | `nav_msgs/msg/Odometry` | Planar odometry adapted from the physical `/state_estimator/odom_pelvis` |
| `/tf` | `tf2_msgs/msg/TFMessage` | `odom -> base_footprint -> pelvis` plus URDF transforms |
| `/scan` | `sensor_msgs/msg/LaserScan` | Passive filtered 2-D projection of physical `/mid360/points` |
| `/map` | `nav_msgs/msg/OccupancyGrid` | Online map from `hardware_mapping.launch.py` |

## Safety behavior

- Motion starts disabled, regardless of incoming `/cmd_vel`.
- Enabling is rejected while `lowstate` is absent or stale.
- A stale `/cmd_vel` causes one explicit zero-velocity request.
- A stale `lowstate` causes one explicit zero-velocity request and blocks further velocity requests.
- Disabling control immediately publishes zero velocity.
- The default limits are deliberately conservative and live in `config/g1_29dof.yaml`.

The bridge uses Unitree high-level locomotion API `7105`; it never publishes `lowcmd` and never takes direct torque control of a motor.

`hardware_telemetry.launch.py` starts `g1_hardware_telemetry_node`, a separate
executable that does not link `unitree_api` and contains no `/cmd_vel`,
`/g1/enable_control`, or `/api/sport/request` command path. The legacy
`hardware_bringup.launch.py` and `bridge.launch.py` files are safe aliases for
that launch.

`hardware_motion.launch.py` is the only physical launch that enables the
high-level locomotion interface. It requires both `motion_interface:=true` and
`allow_hardware_motion:=true`, validates Humble/CycloneDDS/domain 0, and still
starts with `control_enabled=false`.

`hardware_mapping.launch.py` is also passive: it starts only
`/mid360/points -> /scan -> SLAM Toolbox -> /map`. It does not start Nav2,
does not expose `/g1/enable_control`, and has no Unitree command publisher.

## Simulation navigation

These interfaces belong to the Gazebo and MuJoCo simulation stands, not to the
physical G1 bridge:

| Name | Type | Meaning |
|---|---|---|
| `/mid360/points` | `sensor_msgs/msg/PointCloud2` | 3D Mid-360-like cloud raycast from the head-mounted sensor and expressed in simulation `odom` coordinates |
| `/scan` | `sensor_msgs/msg/LaserScan` | 2D navigation projection of the Mid-360 cloud |
| `/odom` | `nav_msgs/msg/Odometry` | Simulation odometry; Gazebo uses `odom -> pelvis`, MuJoCo uses `odom -> base_footprint -> pelvis` with a kinematic base |
| `/map` | `nav_msgs/msg/OccupancyGrid` | Online map from SLAM Toolbox |
| `/plan` | `nav_msgs/msg/Path` | Current Nav2 global plan |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Final Nav2/collision-monitor velocity command to the active simulator |
| `/g1/sim/api/sport/request` | `unitree_api/msg/Request` | Optional simulated G1 high-level locomotion API input; never the native robot topic |
| `/g1/sim/api/sport/response` | `unitree_api/msg/Response` | Optional simulated response to high-level locomotion API calls |

The physical Mid-360 and physical odometry need hardware-specific drivers and
must not be replaced with these simulation sources. In MuJoCo, `/cmd_vel` is a
navigation-interface test path rather than a dynamic walking controller.

When MuJoCo is launched with `loco_api:=true`, `g1_loco_api_sim` accepts a
small hardware-compatible subset of G1 LocoClient requests: FSM/state queries,
`SetFsmId`, `SetBalanceMode`, `SetSwingHeight`, `SetStandHeight`,
`SetSpeedMode`, `SetVelocity` (`7105`), and stop/stand/move aliases from the
generic sport API. This emulates the ROS 2 wire contract only; it is not the
closed firmware locomotion controller from the physical robot.

For a full Nav2-through-API simulation, launch `scripts/mujoco_loco_nav_up.sh`.
That route keeps MuJoCo off direct `/cmd_vel` and uses:

```text
Nav2 /cmd_vel
  -> g1_cmd_vel_loco_bridge
  -> /g1/sim/api/sport/request
  -> g1_loco_api_sim
  -> /g1/sim/cmd_vel
  -> MuJoCo
```

## Real DEX3-1 hands

`dex3_bridge_node` is separate from locomotion and starts disarmed.

| Name | Type | Meaning |
|---|---|---|
| `/lf/dex3/left/state`, `/lf/dex3/right/state` | `unitree_hg/msg/HandState` | Native feedback from each real hand |
| `/dex3/left/cmd`, `/dex3/right/cmd` | `unitree_hg/msg/HandCmd` | Native 7-motor commands sent to each hand |
| `/g1/dex3/{left,right}/command` | `sensor_msgs/msg/JointState` | Seven target motor positions in radians, ordered motor 0 through 6 |
| `/g1/dex3/{left,right}/joint_states` | `sensor_msgs/msg/JointState` | Measured position, velocity and effort |
| `/g1/dex3/enable_control` | `std_srvs/srv/SetBool` | Arm/disarm real hand output |
| `/g1/dex3/stop` | `std_srvs/srv/Trigger` | Disarm both hands and send Unitree timeout commands |
| `/g1/dex3/control_enabled` | `std_msgs/msg/Bool` | Latched DEX3 arm state |

Targets are clamped to the official left/right DEX3-1 limits and rate-limited.
Enabling requires fresh feedback (both hands by default). A stale command stops that
hand; stale feedback disarms the bridge and stops both. Commands received while
disarmed are discarded, so arming cannot replay an old grasp.
