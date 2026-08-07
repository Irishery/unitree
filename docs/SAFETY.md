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

