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

## Safety behavior

- Motion starts disabled, regardless of incoming `/cmd_vel`.
- Enabling is rejected while `lowstate` is absent or stale.
- A stale `/cmd_vel` causes one explicit zero-velocity request.
- A stale `lowstate` causes one explicit zero-velocity request and blocks further velocity requests.
- Disabling control immediately publishes zero velocity.
- The default limits are deliberately conservative and live in `config/g1_29dof.yaml`.

The bridge uses Unitree high-level locomotion API `7105`; it never publishes `lowcmd` and never takes direct torque control of a motor.

