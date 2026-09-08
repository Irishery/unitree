#!/usr/bin/env python3
"""Dev-only: run one grasp profile with continuous IK seeding and PNG frames."""
import os
import sys
from pathlib import Path

import mujoco
import numpy as np

from g1_mujoco.grasp_planning import (
    ARM_JOINTS, FINGER_OPEN, GraspKinematics, GraspParams, WAIST_JOINTS,
    blend_rotation, finger_targets, grasp_frames, rotation_y)
from g1_mujoco.pick_env import CONTROL_JOINTS
from g1_mujoco.sim import ARMS_AT_SIDES
from g1_mujoco.box_geometry import backproject, fit_box_pose, red_box_mask, transform_points

OUT = Path(os.environ.get("G1_PROBE_OUT", "/ws/debug_frames"))
READY = {
    # IK of wrists at x=0.05, y=+/-0.40, z=0.95 using the calibrated
    # side-pinch rotation.  Hands start outside the table footprint.
    "left": np.array([0.623, 0.886, 0.470, 0.289, -0.415, -0.445, -0.379]),
    "right": np.array([0.623, -0.886, -0.470, 0.289, 0.415, -0.445, 0.379]),
}


def main():
    OUT.mkdir(exist_ok=True)
    for old in OUT.glob("*.png"):
        old.unlink()
    kin = GraspKinematics()
    model = kin.model
    for geom_id in range(model.ngeom):
        body = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, model.geom_bodyid[geom_id]) or ""
        if "_hand_" in body:
            model.geom_friction[geom_id, 0] = 4.0
    lateral, rise, reach, twist, scale = (float(v) for v in sys.argv[1:6])
    params = GraspParams(lateral_beyond_face=lateral, rise_above_top=rise,
                         fingertip_reach=reach, palm_tilt_deg=twist,
                         approach_standoff=float(os.environ.get(
                             "G1_APPROACH_STANDOFF", "0.15")),
                         hover_standoff=float(os.environ.get(
                             "G1_HOVER_STANDOFF", "0.15")),
                         finger_close_scale=scale, lift_height=0.16,
                         lift_forward=float(os.environ.get("G1_LIFT_FORWARD", "0.03")),
                         lift_pitch_deg=float(os.environ.get("G1_LIFT_PITCH_DEG", "0")))
    box_yaw = float(sys.argv[6]) if len(sys.argv) > 6 else 0.0

    data = mujoco.MjData(model)
    sim_joints = list(CONTROL_JOINTS) + list(WAIST_JOINTS)
    jid = {n: model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n)] for n in sim_joints}
    vid = {n: model.jnt_dofadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n)] for n in sim_joints}
    aid = {n: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, n) for n in sim_joints}
    for n, v in ARMS_AT_SIDES.items():
        if n in jid:
            data.qpos[jid[n]] = v
    box_q = model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "pickup_box_free")]
    box_body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "pickup_box")
    box_geom = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "pickup_box_geom")
    box_x = float(sys.argv[7]) if len(sys.argv) > 7 else 0.40
    box_y = float(sys.argv[8]) if len(sys.argv) > 8 else 0.0
    data.qpos[box_q:box_q + 7] = [box_x, box_y, 0.80,
                                  np.cos(box_yaw * 0.5), 0.0, 0.0, np.sin(box_yaw * 0.5)]
    mujoco.mj_forward(model, data)
    target = {n: data.qpos[jid[n]] for n in sim_joints}
    goal = {}

    # Runtime input is rendered RGB-D.  The free-joint state above is used
    # only below to report detector and trial errors.
    renderer = mujoco.Renderer(model, width=640, height=480)
    renderer.update_scene(data, camera="d435i")
    rgb = renderer.render()
    renderer.enable_depth_rendering()
    depth = renderer.render().astype(np.float32)
    renderer.disable_depth_rendering()
    fy = 480.0 / (2.0 * np.tan(np.deg2rad(69.0) * 0.5))
    points, _ = backproject(red_box_mask(rgb), depth, fy, fy, 320.0, 240.0)
    camera_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, "d435i")
    camera_rotation = data.cam_xmat[camera_id].reshape(3, 3) @ np.diag([1.0, -1.0, -1.0])
    points = transform_points(points, camera_rotation, data.cam_xpos[camera_id])
    detected = fit_box_pose(points)
    if detected is None:
        raise RuntimeError("RGB-D box detection failed")
    centre, detected_yaw = detected["centre"], detected["yaw"]
    yaw_error = min(abs(np.arctan2(np.sin(detected_yaw - box_yaw), np.cos(detected_yaw - box_yaw))),
                    abs(np.pi - abs(np.arctan2(np.sin(detected_yaw - box_yaw), np.cos(detected_yaw - box_yaw)))))
    print("RGBD detected", centre.round(4), "yaw", round(detected_yaw, 4),
          "GT errors", round(float(np.linalg.norm(centre - [box_x, box_y, .8])), 4),
          round(float(np.degrees(yaw_error)), 2), "deg")
    frames = grasp_frames(centre, detected_yaw, params)
    arm_q = {}
    ingress_q = {}
    slide_q = {}
    lift_q = {}
    seat_q = {}
    for side in ("left", "right"):
        sign = 1.0 if side == "left" else -1.0
        seed = READY[side]
        ingress_q[side] = []
        start_pos = np.array([0.05, sign * 0.40, 0.95])
        for alpha in np.linspace(0.2, 1.0, 5):
            position = start_pos * (1.0 - alpha) + frames[side]["pregrasp"] * alpha
            seed = kin.solve_arm_ik(
                side, position, frames[side]["approach_rotation"], q_init=seed)
            ingress_q[side].append(seed)
        arm_q.setdefault(side, {})["pregrasp"] = ingress_q[side][-1]
        arm_q.setdefault(side, {})["hover"] = kin.solve_arm_ik(
            side, frames[side]["hover"], frames[side]["approach_rotation"], q_init=seed)
        seed = arm_q[side]["hover"]
        slide_q[side] = []
        for alpha in np.linspace(0.125, 1.0, 8):
            position = ((1.0 - alpha) * frames[side]["hover"]
                        + alpha * frames[side]["grasp"])
            rotation = blend_rotation(
                frames[side]["approach_rotation"], frames[side]["rotation"], alpha)
            seed = kin.solve_arm_ik(side, position, rotation, q_init=seed)
            slide_q[side].append(seed)
        arm_q[side]["grasp"] = slide_q[side][-1]
        lift_q[side] = []
        for dz in np.linspace(0.02, params.lift_height, 6):
            lift_alpha = dz / params.lift_height
            lift_rotation = (rotation_y(np.deg2rad(params.lift_pitch_deg) * lift_alpha)
                             @ frames[side]["rotation"])
            seed = kin.solve_arm_ik(
                side,
                frames[side]["grasp"]
                + np.array([params.lift_forward * lift_alpha, 0.0, dz]),
                lift_rotation, q_init=seed)
            lift_q[side].append(seed)
        arm_q[side]["lift"] = lift_q[side][-1]
        seat_q[side] = arm_q[side]["grasp"]
        print(side, "wrist grasp target", frames[side]["grasp"].round(3),
              "IK residual", tuple(round(v, 4) for v in kin.last_ik_residual),
              "q_grasp", arm_q[side]["grasp"].round(2))
        for stage in ("pregrasp", "hover", "grasp", "lift"):
            print("  ", stage, arm_q[side][stage].round(3))

    hand_names = {side: [n for n in CONTROL_JOINTS if n.startswith(f"{side}_hand_")]
                  for side in ("left", "right")}
    camera = mujoco.MjvCamera()
    camera.lookat = [0.42, 0.0, 0.88]
    camera.distance = 1.35
    camera.elevation = -18
    camera.azimuth = 135
    frame_index = [0]
    tilt_log = {}

    def snapshot(label):
        renderer.update_scene(data, camera)
        image = renderer.render()
        from PIL import Image
        Image.fromarray(image).save(OUT / f"{frame_index[0]:02d}_{label}.png")
        frame_index[0] += 1

    def robot_box_contacts():
        count, bodies = 0, set()
        for index in range(data.ncon):
            contact = data.contact[index]
            b1 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, model.geom_bodyid[contact.geom1]) or ""
            b2 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, model.geom_bodyid[contact.geom2]) or ""
            pair = {b1, b2}
            if "pickup_box" in pair and (pair - {"pickup_box", "table", "world"}):
                count += 1
                bodies |= pair - {"pickup_box", "table", "world"}
        return count, bodies

    def box_tilt_deg():
        rotation = data.xmat[box_body].reshape(3, 3)
        return float(np.degrees(np.arccos(np.clip(rotation[2, 2], -1.0, 1.0))))

    def box_pitch_deg():
        rotation = data.xmat[box_body].reshape(3, 3)
        return float(np.degrees(np.arctan2(-rotation[2, 0],
                                           np.hypot(rotation[2, 1], rotation[2, 2]))))

    def box_contact_details():
        details = []
        box_rotation = data.xmat[box_body].reshape(3, 3)
        box_centre_now = data.xpos[box_body]
        for index in range(data.ncon):
            contact = data.contact[index]
            body1 = mujoco.mj_id2name(
                model, mujoco.mjtObj.mjOBJ_BODY, model.geom_bodyid[contact.geom1]) or ""
            body2 = mujoco.mj_id2name(
                model, mujoco.mjtObj.mjOBJ_BODY, model.geom_bodyid[contact.geom2]) or ""
            if "pickup_box" not in (body1, body2):
                continue
            hand = body2 if body1 == "pickup_box" else body1
            if "_hand_" not in hand and "wrist" not in hand:
                continue
            local = box_rotation.T @ (contact.pos - box_centre_now)
            force = np.zeros(6)
            mujoco.mj_contactForce(model, data, index, force)
            details.append((hand, np.round(local, 3).tolist(), round(float(force[0]), 2)))
        return details

    def hand_table_contacts():
        bodies = set()
        for index in range(data.ncon):
            contact = data.contact[index]
            b1 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, model.geom_bodyid[contact.geom1]) or ""
            b2 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, model.geom_bodyid[contact.geom2]) or ""
            if "table" in (b1, b2) and "_hand_" in (b1 + b2):
                bodies |= {b for b in (b1, b2) if "_hand_" in b}
        return sorted(bodies)

    def step(seconds, rate=1.6, label="step"):
        max_contacts = 0
        heights = []
        tilts = []
        for step_index in range(int(seconds / 0.002)):
            data.qpos[0:3] = [0.0, 0.0, 0.793]
            data.qpos[3:7] = [1.0, 0.0, 0.0, 0.0]
            data.qvel[0:6] = 0.0
            for n in sim_joints:
                finger = "_hand_" in n
                if finger:
                    kp = float(os.environ.get("G1_FINGER_KP", "12.0"))
                    kd = float(os.environ.get("G1_FINGER_KD", "0.45"))
                    tl = float(os.environ.get("G1_FINGER_TORQUE_LIMIT", "3.0"))
                else:
                    kp, kd, tl = 80.0, 3.0, 35.0
                if n in goal:
                    desired = min(target[n] + rate * 0.002, goal[n]) if goal[n] >= target[n] \
                        else max(target[n] - rate * 0.002, goal[n])
                    target[n] = desired
                data.ctrl[aid[n]] = max(-tl, min(tl, kp * (target[n] - data.qpos[jid[n]]) - kd * data.qvel[vid[n]]))
            mujoco.mj_step(model, data)
            # Match G1Mujoco.step(): the tabletop benchmark pins the floating
            # base both before and after physics, then refreshes kinematics.
            data.qpos[0:3] = [0.0, 0.0, 0.793]
            data.qpos[3:7] = [1.0, 0.0, 0.0, 0.0]
            data.qvel[0:6] = 0.0
            mujoco.mj_forward(model, data)
            heights.append(data.xpos[box_body][2])
            tilts.append(box_tilt_deg())
            if step_index % 25 == 0:
                max_contacts = max(max_contacts, robot_box_contacts()[0])
        count, bodies = robot_box_contacts()
        tilt_log[label] = tilts
        print(f"{label}: box xyz {data.xpos[box_body].round(3).tolist()} zmin {min(heights):.3f} "
              f"tilt_max {max(tilts):.2f}deg pitch {box_pitch_deg():.2f}deg "
              f"contacts_during {max_contacts} end {count} {sorted(bodies)[:3]}")
        if hand_table_contacts():
            print("   hand-table", hand_table_contacts())
        if label == "close" or label.startswith("lift_") or label == "hold_30s":
            print("   box contacts body/local_xyz", box_contact_details())
        for side in ("left", "right"):
            tips = []
            for finger in ("index_1", "middle_1", "thumb_2"):
                body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f"{side}_hand_{finger}_link")
                tips.append(data.xpos[body].round(3).tolist())
            print(f"   {side} tips idx/mid/thb {tips}")
            print(f"   {side} arm q {[round(float(data.qpos[jid[n]]), 3) for n in ARM_JOINTS[side]]}")
            print(f"   {side} wrist {data.xpos[kin.wrist_ids[side]].round(3).tolist()}")
        return np.array(heights)

    def set_joint_goal(stage, close_fingers=False, hold_arms=None):
        for side in ("left", "right"):
            for name, value in zip(ARM_JOINTS[side], arm_q[side][hold_arms or stage]):
                goal[name] = float(value)
            for name, value in zip(hand_names[side],
                                   finger_targets(side, params.finger_close_scale, params)
                                   if close_fingers else FINGER_OPEN[side]):
                goal[name] = float(value)

    def ready_pose():
        return READY

    snapshot("start")
    q_ready = ready_pose()
    for side in ("left", "right"):
        for name, value in zip(ARM_JOINTS[side], q_ready[side]):
            goal[name] = float(value)
        for name, value in zip(hand_names[side], FINGER_OPEN[side]):
            goal[name] = float(value)
    step(2.5, label="ready")
    snapshot("ready")
    for index in range(5):
        for side in ("left", "right"):
            for name, value in zip(ARM_JOINTS[side], ingress_q[side][index]):
                goal[name] = float(value)
        step(0.7, rate=0.7, label=f"ingress_{index + 1}")
    step(1.0, label="pregrasp")
    snapshot("pregrasp")
    set_joint_goal("hover")
    step(1.5, label="hover")
    snapshot("hover")
    for index in range(8):
        for side in ("left", "right"):
            for name, value in zip(ARM_JOINTS[side], slide_q[side][index]):
                goal[name] = float(value)
        step(0.6, rate=0.55, label=f"side_clamp_{index + 1}")
    snapshot("grasp")
    set_joint_goal("grasp", close_fingers=True)
    step(2.0, rate=6.0, label="close")
    snapshot("close")
    if os.environ.get("G1_PROBE_STOP_AFTER_CLOSE") == "1":
        renderer.close()
        return
    heights = []
    for index in range(6):
        for side in ("left", "right"):
            for name, value in zip(ARM_JOINTS[side], lift_q[side][index]):
                goal[name] = float(value)
            for name, value in zip(hand_names[side], finger_targets(side, params.finger_close_scale, params)):
                goal[name] = float(value)
        heights.extend(step(float(os.environ.get("G1_LIFT_SEGMENT_SECONDS", "2.5")),
                            rate=float(os.environ.get("G1_LIFT_RATE", "0.15")),
                            label=f"lift_{index + 1}"))
        # Keep visual evidence for partial probe runs as well.  Previously a
        # lift image was written only after all six stages and the 30-second
        # hold, so STOP_AFTER_LIFT artifacts ended at close.png.
        snapshot(f"lift_{index + 1}")
        stop_after = int(os.environ.get("G1_PROBE_STOP_AFTER_LIFT", "0"))
        if stop_after and index + 1 >= stop_after:
            renderer.close()
            return
    heights.extend(step(15.0, rate=0.7, label="settle_before_hold"))
    hold_heights = step(30.0, rate=0.7, label="hold_30s")
    hold_tilts = np.asarray(tilt_log["hold_30s"])
    heights.extend(hold_heights)
    heights = np.asarray(heights)
    snapshot("lift")
    print(f"HOLD box z start {hold_heights[0]:.3f} min {hold_heights.min():.3f} "
          f"end {hold_heights[-1]:.3f} slip {hold_heights.max() - hold_heights.min():.3f} "
          f"tilt_max_deg {hold_tilts.max():.2f} tilt_end_deg {hold_tilts[-1]:.2f}")
    for index in reversed(range(6)):
        target_index = max(0, index - 1)
        for side in ("left", "right"):
            for name, value in zip(ARM_JOINTS[side],
                                   lift_q[side][target_index] if index else arm_q[side]["grasp"]):
                goal[name] = float(value)
        step(0.8, rate=0.55, label=f"place_{6 - index}")
    step(1.5, rate=0.4, label="place_settle")
    for side in ("left", "right"):
        for name, value in zip(ARM_JOINTS[side], seat_q[side]):
            goal[name] = float(value)
    step(2.0, rate=0.45, label="seat")
    set_joint_goal("grasp", close_fingers=False, hold_arms=None)
    for side in ("left", "right"):
        for name, value in zip(ARM_JOINTS[side], seat_q[side]):
            goal[name] = float(value)
    step(1.0, rate=1.0, label="release")
    for index in reversed(range(5)):
        for side in ("left", "right"):
            for name, value in zip(ARM_JOINTS[side], ingress_q[side][index]):
                goal[name] = float(value)
        step(0.7, rate=0.6, label=f"retreat_{5-index}")
    for side in ("left", "right"):
        for name, value in zip(ARM_JOINTS[side], READY[side]):
            goal[name] = float(value)
    step(2.0, rate=0.6, label="ready_after")
    step(3.0, label="placed_settle")
    print(f"FINAL box z max {heights.max():.3f} hold_end {hold_heights[-1]:.3f} "
          f"placed {data.xpos[box_body][2]:.3f}")
    renderer.close()


if __name__ == "__main__":
    main()
