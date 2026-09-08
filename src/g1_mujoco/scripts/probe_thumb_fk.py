#!/usr/bin/env python3
"""Print compensated DEX3 thumb poses without stepping box physics."""
import numpy as np

from g1_mujoco.grasp_planning import (
    HAND_JOINTS, GraspKinematics, GraspParams, grasp_frames)


def main():
    kin = GraspKinematics()
    params = GraspParams(fingertip_reach=0.21, approach_reach=0.21,
                         lateral_beyond_face=-0.01, rise_above_top=-0.01,
                         palm_tilt_deg=10.0)
    frames = grasp_frames(np.array([0.4, 0.0, 0.8]), 0.0, params)
    arms = {}
    for side in ("left", "right"):
        arms[side] = kin.solve_arm_ik(
            side, frames[side]["grasp"], frames[side]["rotation"])
    joints = {name: value for side in arms
              for name, value in zip(kin_names(side), arms[side])}
    for degrees in (0, 5, 10, 15, 20, 25, 30, 35):
        bend = np.deg2rad(degrees)
        for side, mirror in (("left", 1.0), ("right", -1.0)):
            values = np.array([0.0, -bend * mirror, bend * mirror,
                               -0.04 * mirror, -0.04 * mirror,
                               -0.04 * mirror, -0.04 * mirror])
            for name, value in zip(HAND_JOINTS[side], values):
                joints[name] = value
        kin.set_robot_state(joints)
        values = []
        for side in ("left", "right"):
            body = kin.model.body(f"{side}_hand_thumb_2_link").id
            origin = kin.data.xpos[body]
            rotation = kin.data.xmat[body].reshape(3, 3)
            tip = origin - rotation[:, 1] * 0.045
            values.append((side, np.round(origin, 4).tolist(),
                           np.round(tip, 4).tolist(),
                           np.round(-rotation[:, 1], 3).tolist()))
        print(f"bend={degrees:2d}deg", values)


def kin_names(side):
    from g1_mujoco.grasp_planning import ARM_JOINTS
    return ARM_JOINTS[side]


if __name__ == "__main__":
    main()
