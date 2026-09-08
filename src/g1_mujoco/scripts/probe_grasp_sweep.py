#!/usr/bin/env python3
"""Dev-only numeric sweep that tunes the bimanual grasp parameters.

Replicates the tabletop executor PD loop (arm 35/1.8/18, DEX3 3/0.18/0.9,
pinned floating base) directly in MuJoCo so candidate grasp geometries can be
scored by contact physics before any ROS wiring.  Not part of the runtime.
"""
import sys
import numpy as np

from g1_mujoco.grasp_planning import (
    ARM_JOINTS, FINGER_OPEN, GraspKinematics, GraspParams, finger_targets, grasp_frames)
from g1_mujoco.pick_env import CONTROL_JOINTS
from g1_mujoco.sim import ARMS_AT_SIDES

import mujoco


def run_combo(model, kin, lateral, rise, reach, twist, scale, box_yaw=0.0, verbose=False):
    data = mujoco.MjData(model)
    jid = {n: model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n)] for n in CONTROL_JOINTS}
    vid = {n: model.jnt_dofadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n)] for n in CONTROL_JOINTS}
    aid = {n: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, n) for n in CONTROL_JOINTS}
    for n, v in ARMS_AT_SIDES.items():
        if n in jid:
            data.qpos[jid[n]] = v
    box_q = model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "pickup_box_free")]
    box_body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "pickup_box")
    data.qpos[box_q:box_q + 7] = [0.45, 0.0, 0.80, 1.0, 0.0, 0.0, 0.0]
    mujoco.mj_forward(model, data)
    target = {n: data.qpos[jid[n]] for n in CONTROL_JOINTS}
    goal = {}

    params = GraspParams(lateral_beyond_face=lateral, rise_above_top=rise,
                         fingertip_reach=reach, palm_tilt_deg=twist,
                         finger_close_scale=scale)
    centre = np.array([0.45, 0.0, 0.80])
    frames = grasp_frames(centre, box_yaw, params)
    kin.set_box(centre, box_yaw)
    arm_q = {}
    try:
        for side in ("left", "right"):
            arm_q[side] = {stage: kin.solve_arm_ik(side, frames[side][stage], frames[side]["rotation"])
                           for stage in ("pregrasp", "hover", "grasp", "lift")}
    except RuntimeError as error:
        return {"ik": f"fail: {error}"}

    def hand_contacts():
        count = 0
        sides = set()
        for index in range(data.ncon):
            contact = data.contact[index]
            b1 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, model.geom_bodyid[contact.geom1]) or ""
            b2 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, model.geom_bodyid[contact.geom2]) or ""
            if "pickup_box" in (b1, b2) and "_hand_" in (b1 + b2):
                count += 1
                sides.add("left" if "left" in (b1 + b2) else "right")
        return count, sides

    def step(seconds, rate=1.6):
        heights = []
        for _ in range(int(seconds / 0.002)):
            data.qpos[0:3] = [0.0, 0.0, 0.793]
            data.qpos[3:7] = [1.0, 0.0, 0.0, 0.0]
            data.qvel[0:6] = 0.0
            for n in CONTROL_JOINTS:
                finger = "_hand_" in n
                kp, kd, tl = (3.0, 0.18, 0.9) if finger else (35.0, 1.8, 18.0)
                if n in goal:
                    desired = min(target[n] + rate * 0.002, goal[n]) if goal[n] >= target[n] \
                        else max(target[n] - rate * 0.002, goal[n])
                    target[n] = desired
                data.ctrl[aid[n]] = max(-tl, min(tl, kp * (target[n] - data.qpos[jid[n]]) - kd * data.qvel[vid[n]]))
            mujoco.mj_step(model, data)
            heights.append(data.xpos[box_body][2])
        return np.array(heights)

    def set_stage(stage, close_fingers=False):
        arm_stage = "grasp" if stage == "close" else stage
        for side in ("left", "right"):
            for name, value in zip(ARM_JOINTS[side], arm_q[side][arm_stage]):
                goal[name] = float(value)
        for side in ("left", "right"):
            for name, value in zip(HAND_NAMES[side],
                                   finger_targets(side, params.finger_close_scale, params)
                                   if close_fingers else FINGER_OPEN[side]):
                goal[name] = float(value)

    HAND_NAMES = {side: [n for n in CONTROL_JOINTS if n.startswith(f"{side}_hand_")]
                  for side in ("left", "right")}
    result = {}
    set_stage("pregrasp")
    step(3.0)
    result["pregrasp_contacts"] = hand_contacts()[0]
    set_stage("hover")
    step(1.5)
    result["hover_contacts"] = hand_contacts()[0]
    set_stage("grasp")
    heights = step(1.5)
    result["grasp_contacts"] = hand_contacts()
    set_stage("close", close_fingers=True)
    heights = step(2.0, rate=6.0)
    result["close_contacts"] = hand_contacts()
    result["close_box_z"] = float(heights[-1])
    set_stage("lift")
    heights = step(2.5)
    result["lift_contacts"] = hand_contacts()
    result["lift_box_max"] = float(heights.max())
    result["lift_box_end"] = float(heights[-1])
    if verbose:
        for side in ("left", "right"):
            print(side, "wrist", data.xpos[kin.wrist_ids[side]].round(3))
    return result


def main():
    kin = GraspKinematics()
    model = kin.model
    combos = [(lateral, rise, reach, twist, scale)
              for lateral in (0.06, 0.08)
              for rise in (0.07, 0.09)
              for reach in (0.15, 0.16)
              for twist in (-10.0, 0.0, 10.0)
              for scale in (1.0, 1.3)]
    print(f"{'lat':>5} {'rise':>5} {'reach':>5} {'tilt':>4} {'scale':>5} | "
          f"{'pre':>3} {'hov':>3} {'grasp':>5} {'close':>5} {'lift':>5} | {'z_close':>7} {'z_max':>6} {'z_end':>6}")
    for lateral, rise, reach, twist, scale in combos:
        try:
            r = run_combo(model, kin, lateral, rise, reach, twist, scale)
        except Exception as error:  # noqa: BLE001 - dev sweep keeps going
            print(f"{lateral:5.2f} {rise:5.2f} {reach:5.2f} {twist:4.0f} {scale:5.2f} | ERROR {error}")
            continue
        if "ik" in r:
            print(f"{lateral:5.2f} {rise:5.2f} {reach:5.2f} {twist:4.0f} {scale:5.2f} | IK {r['ik'][:45]}")
            continue
        grasp_c, grasp_s = r["grasp_contacts"]
        close_c, close_s = r["close_contacts"]
        lift_c, lift_s = r["lift_contacts"]
        print(f"{lateral:5.2f} {rise:5.2f} {reach:5.2f} {twist:4.0f} {scale:5.2f} | "
              f"{r['pregrasp_contacts']:3d} {r['hover_contacts']:3d} "
              f"{grasp_c:3d}{'L' if 'left' in grasp_s else ''}{'R' if 'right' in grasp_s else ''} "
              f"{close_c:3d}{'L' if 'left' in close_s else ''}{'R' if 'right' in close_s else ''} "
              f"{lift_c:3d}{'L' if 'left' in lift_s else ''}{'R' if 'right' in lift_s else ''} | "
              f"{r['close_box_z']:7.3f} {r['lift_box_max']:6.3f} {r['lift_box_end']:6.3f}")


if __name__ == "__main__":
    sys.exit(main())
