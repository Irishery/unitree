# First physical-robot run

1. Use a G1 EDU supported by SDK2. Keep the original wireless controller in hand.
2. Put the robot on its safety gantry/support for the first motion test.
3. Clear people, cables, and furniture from the full fall radius.
4. Confirm the firmware's normal high-level locomotion service is active. Do not mix this bridge with a second program that publishes locomotion commands.
5. Start the bridge and verify `/g1/imu/data`, `/g1/joint_states`, and `/diagnostics` before enabling control.
6. Enable with `/g1/enable_control`, then start at `0.05 m/s` for less than one second.
7. Verify that releasing the teleop key stops the robot within the configured watchdog interval.
8. Verify that disabling the service stops commands before trying higher limits.

Never test walking for the first time in low-level/debug motor-control mode. Firmware and remote-controller key combinations differ between G1 releases; use the instructions shipped with the robot to select normal locomotion mode.

## DEX3-1 hands

- Test with both hands empty, fingers clear of people/cables and the G1 supported.
- Confirm `/lf/dex3/left/state` and `/lf/dex3/right/state` before enabling.
- Start with the configured `kp: 0.5`, `max_velocity_rad_s: 0.5`; increase neither
  until the motor order, limits and firmware are verified on the exact hardware.
- Keep publishing commands while motion is intended. The default 0.35 s watchdog
  deliberately releases motor control when the GUI or network stops.
- `/g1/dex3/stop` is the software stop; keep the physical Unitree stop available.
- Do not run Unitree's DEX3 example or another hand controller at the same time.
