# Walking policy (g1_12dof_motion.pt)

Pre-trained G1 locomotion policy from Unitree's official
[unitree_rl_gym](https://github.com/unitreerobotics/unitree_rl_gym)
repository (`deploy/pre_train/g1/motion.pt`), trained with legged_gym /
Isaac Gym for the 12-DoF leg configuration.

- Input: 47-dim observation (base ang vel, gravity orientation, velocity
  command, 12 joint pos/vel, previous action, gait phase sin/cos).
- Output: 12 leg position targets (action_scale 0.25 around the default
  pose), tracked by joint PD at 500 Hz with per-joint gains from
  `deploy/deploy_mujoco/configs/g1.yaml`.
- Velocity commands up to +-1.0 m/s (x, y) and +-1 rad/s (yaw) with
  friction/mass/push domain randomization.

License: BSD 3-Clause (Unitree Robotics), see
`LICENSE.unitree_rl_gym`. Used in `g1_mujoco` as the `walk:=true`
locomotion backend; the deployment parameters are reproduced in
`src/g1_mujoco/g1_mujoco/sim.py`.
