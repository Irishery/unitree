"""Kinematics, IK and grasp target planning for the bimanual DEX3 pick task.

The planner is deliberately free of ROS and of live simulator state: it owns a
private read-only MuJoCo model used only for forward kinematics, Jacobians and
collision-distance queries.  A hardware deployment swaps this class for a
URDF-based kinematics provider with the same interface; the scenario logic in
dual_pick_controller.py never touches simulator signals directly.
"""
from dataclasses import dataclass, field, replace
from pathlib import Path
import os
import math

import mujoco
import numpy as np


ARM_JOINTS = {
    "left": ["left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
             "left_elbow_joint", "left_wrist_roll_joint", "left_wrist_pitch_joint", "left_wrist_yaw_joint"],
    "right": ["right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
              "right_elbow_joint", "right_wrist_roll_joint", "right_wrist_pitch_joint", "right_wrist_yaw_joint"],
}
HAND_JOINTS = {
    "left": ["left_hand_thumb_0_joint", "left_hand_thumb_1_joint", "left_hand_thumb_2_joint",
             "left_hand_middle_0_joint", "left_hand_middle_1_joint",
             "left_hand_index_0_joint", "left_hand_index_1_joint"],
    "right": ["right_hand_thumb_0_joint", "right_hand_thumb_1_joint", "right_hand_thumb_2_joint",
              "right_hand_middle_0_joint", "right_hand_middle_1_joint",
              "right_hand_index_0_joint", "right_hand_index_1_joint"],
}
# Waist and base are held at their zero pose by the executor in the tabletop
# stand; the planner needs the same assumptions for consistent IK.
WAIST_JOINTS = ["waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint"]
BASE_POSE = np.array([0.0, 0.0, 0.793, 1.0, 0.0, 0.0, 0.0])

# Nullspace posture bias: arms forward with folded elbows keeps the IK away
# from singularity and self-collision.
POSTURE_BIAS = {
    "left": [-0.70, 0.45, -0.15, 0.90, 0.0, 0.0, 0.0],
    "right": [-0.70, -0.45, 0.15, 0.90, 0.0, 0.0, 0.0],
}

MAX_IK_ITERS = 300
IK_POS_TOL = 3.0e-3
IK_ROT_TOL = 4.0 * math.pi / 180.0
# Intermediate outside-the-box waypoints may lie a few millimetres beyond the
# exact 6D workspace of a straight wrist.  The final grasp solutions remain
# below the stricter IK_POS_TOL whenever reachable.
IK_POS_ACCEPT = 25.0e-3
IK_ROT_ACCEPT = 15.0 * math.pi / 180.0
IK_STEP_MAX = 0.12
CLEARANCE_MARGIN = 8.0e-3

# Shared joint-range cache filled by GraspKinematics so pure helpers such as
# finger_targets can clamp without loading their own model.
JOINT_RANGES = {}

# Upright palm frame for the broad side clamp.  The DEX3 palm and both straight
# fingers lie in the local X-Z plane, so local Y is the side-face normal.  The
# right-hand meshes are already mirrored in MJCF and use the same wrist frame.
BASELINE_WRIST_AXES = {
    "left": {
        "x": (1.0, 0.0, 0.0),
        "y": (0.0, 0.984808, 0.173648),
        "z": (0.0, -0.173648, 0.984808),
    },
    "right": {
        "x": (1.0, 0.0, 0.0),
        "y": (0.0, 0.984808, -0.173648),
        "z": (0.0, 0.173648, 0.984808),
    },
}

# While the hands travel past the front edge, cant the top of each palm away
# from the box.  This makes room for the DEX3 thumb, whose open linkage still
# projects inward.  The final lateral slide simultaneously returns the palms
# to the near-upright frame above.
APPROACH_WRIST_AXES = {
    "left": {
        "x": (1.0, 0.0, 0.0),
        "y": (0.0, 0.866025, -0.5),
        "z": (0.0, 0.5, 0.866025),
    },
    "right": {
        "x": (1.0, 0.0, 0.0),
        "y": (0.0, 0.866025, 0.5),
        "z": (0.0, -0.5, 0.866025),
    },
}


def axis_angle_from_matrix(rotation):
    trace = np.clip((np.trace(rotation) - 1.0) * 0.5, -1.0, 1.0)
    angle = math.acos(float(trace))
    if angle < 1e-9:
        return np.zeros(3)
    if angle > math.pi - 1e-6:
        # Near pi: use the symmetric part to recover the axis.
        column = 0.5 * (np.diag(rotation) + np.ones(3))
        axis = np.sqrt(np.clip(column, 0.0, None))
        if rotation[0, 1] < 0.0:
            axis[1] *= -1.0
        if rotation[0, 2] < 0.0:
            axis[2] *= -1.0
        return axis * angle
    axis = np.array([rotation[2, 1] - rotation[1, 2],
                     rotation[0, 2] - rotation[2, 0],
                     rotation[1, 0] - rotation[0, 1]]) / (2.0 * math.sin(angle))
    return axis * angle


def rotation_from_axes(x_axis, y_axis, z_axis):
    rotation = np.column_stack((x_axis, y_axis, z_axis))
    u, _, vt = np.linalg.svd(rotation)
    return u @ vt


def blend_rotation(start, end, alpha):
    """Project a short linear SO(3) blend back onto a rotation matrix."""
    u, _, vt = np.linalg.svd((1.0 - alpha) * start + alpha * end)
    rotation = u @ vt
    if np.linalg.det(rotation) < 0.0:
        u[:, -1] *= -1.0
        rotation = u @ vt
    return rotation


def yaw_quaternion(yaw):
    half = 0.5 * yaw
    return np.array([math.cos(half), 0.0, 0.0, math.sin(half)])


@dataclass
class GraspParams:
    """Tuned grasp geometry; all offsets are in the box frame, metres."""
    box_dims: tuple = (0.150, 0.250, 0.140)
    fingertip_reach: float = 0.140        # straight thumb on the short front panel
    approach_reach: float = 0.170         # keep thumb ahead of front edge until side entry
    lateral_beyond_face: float = 0.010    # initial broad-palm contact before arm preload
    rise_above_top: float = -0.010        # palm centre below the top edge
    approach_standoff: float = 0.10       # clears the inward-projecting open thumb
    hover_standoff: float = 0.10          # stay fully outside during the forward alignment
    align_standoff: float = 0.06          # shift forward while still clear of the side face
    arm_preload: float = 0.030            # symmetric whole-hand squeeze after finger seating
    front_seat: float = 0.0               # no longitudinal push after contact
    # Symmetric correction of the calibrated 10-degree wrist roll.  A value
    # of 10 makes both palms exactly upright (90 degrees to the table).
    palm_tilt_deg: float = 10.0
    lift_height: float = 0.116            # produces 11.4 cm held object rise in the tuned grip
    # Follow the small forward motion caused by transferring the free box's
    # weight from the table to the compliant hands.  This keeps the straight
    # thumb/front-palm support on the front panel during lift.
    lift_forward: float = 0.034
    lift_pitch_deg: float = 0.0          # in-plane wrist pitch applied during lift
    lift_elbow_min: float = 0.0          # do not reverse the elbow branch during carrying
    hold_pitch_deg: float = -15.0        # lean toward the robot and load the straight thumbs
    finger_close_scale: float = 1.0
    # Extra squeeze used only while the base carries the box between tables.
    # The stationary grasp is unchanged; a small inward preload and firmer
    # fingers keep the box from rotating out of the grip during transport.
    transport_squeeze: float = 0.0
    transport_finger_scale: float = 1.0
    transport_pitch_deg: float = -15.0  # validated carry tilt (wrist contacts excluded)


class GraspKinematics:
    """Read-only kinematic model shared by planning and the scenario node."""

    def __init__(self, model_path=None):
        description = Path(os.environ.get("G1_DESCRIPTION_DIR", "/opt/unitree_ros/robots/g1_description"))
        path = str(model_path or os.environ.get("G1_MUJOCO_MODEL")
                   or description / "g1_29dof_with_dex3_tabletop.xml")
        self.model = mujoco.MjModel.from_xml_path(path)
        self.data = mujoco.MjData(self.model)
        self.joint_ids = {}
        self.qadr = {}
        self.vadr = {}
        self.ranges = {}
        for side in ("left", "right"):
            for name in ARM_JOINTS[side] + HAND_JOINTS[side]:
                jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
                self.joint_ids[name] = jid
                self.qadr[name] = int(self.model.jnt_qposadr[jid])
                self.vadr[name] = int(self.model.jnt_dofadr[jid])
                self.ranges[name] = (float(self.model.jnt_range[jid, 0]), float(self.model.jnt_range[jid, 1]))
                JOINT_RANGES[name] = self.ranges[name]
        for name in WAIST_JOINTS:
            jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            if jid >= 0:
                self.joint_ids[name] = jid
                self.qadr[name] = int(self.model.jnt_qposadr[jid])
        self.wrist_ids = {
            side: mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, f"{side}_wrist_yaw_link")
            for side in ("left", "right")
        }
        box_joint = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "pickup_box_free")
        self.box_qadr = int(self.model.jnt_qposadr[box_joint])
        self.box_body = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "pickup_box")
        self._build_collision_pairs()
        self.reset_state()

    def _collision_geoms(self, body_predicate):
        geoms = []
        for geom_id in range(self.model.ngeom):
            body_id = int(self.model.geom_bodyid[geom_id])
            body = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, body_id) or ""
            if not body_predicate(body):
                continue
            contype = int(self.model.geom_contype[geom_id])
            conaffinity = int(self.model.geom_conaffinity[geom_id])
            if contype == 0 and conaffinity == 0:
                continue  # visual-only geom
            geoms.append(geom_id)
        return geoms

    def _build_collision_pairs(self):
        def arm_of(body):
            for side in ("left", "right"):
                if any(body.startswith(f"{side}_{part}") for part in ("shoulder", "elbow", "wrist", "hand")):
                    return side
            return None

        left = self._collision_geoms(lambda b: arm_of(b) == "left")
        right = self._collision_geoms(lambda b: arm_of(b) == "right")
        robot_rest = self._collision_geoms(lambda b: arm_of(b) is None and b not in ("world",))
        table = [mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, "table_top")]
        box = self._collision_geoms(lambda body: body == "pickup_box")
        self.clearance_pairs = []
        for side_geoms, other_geoms, label in (
            (left, right + robot_rest + table + box, "left-vs-rest"),
            (right, left + robot_rest + table + box, "right-vs-rest"),
            (left + right, robot_rest, "arms-vs-body"),
        ):
            self.clearance_pairs.append((side_geoms, other_geoms, label))

    def reset_state(self):
        mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[0:7] = BASE_POSE
        mujoco.mj_forward(self.model, self.data)

    def set_joint(self, name, value):
        if name in self.qadr:
            low, high = self.ranges.get(name, (-math.pi, math.pi))
            self.data.qpos[self.qadr[name]] = min(high, max(low, float(value)))

    def set_robot_state(self, joint_values):
        self.data.qpos[0:7] = BASE_POSE
        for name, value in joint_values.items():
            self.set_joint(name, value)
        mujoco.mj_forward(self.model, self.data)

    def set_box(self, centre, yaw):
        self.data.qpos[self.box_qadr:self.box_qadr + 3] = centre
        self.data.qpos[self.box_qadr + 3:self.box_qadr + 7] = yaw_quaternion(yaw)
        mujoco.mj_forward(self.model, self.data)

    def wrist_pose(self, side):
        body = self.wrist_ids[side]
        return self.data.xpos[body].copy(), self.data.xmat[body].reshape(3, 3).copy()

    def arm_positions(self, side):
        return np.array([self.data.qpos[self.qadr[name]] for name in ARM_JOINTS[side]])

    def min_clearance(self, distmax=0.08):
        mujoco.mj_forward(self.model, self.data)
        worst = distmax
        label = None
        for geoms_a, geoms_b, name in self.clearance_pairs:
            for g1 in geoms_a:
                for g2 in geoms_b:
                    if g1 == g2:
                        continue
                    distance = mujoco.mj_geomDistance(self.model, self.data, g1, g2, distmax)
                    if distance < worst:
                        worst, label = distance, name
        return worst, label

    def solve_arm_ik(self, side, target_pos, target_rot, q_init=None,
                     joint_bounds=None):
        """Damped least-squares IK for one 7-DoF arm with restarts."""
        names = ARM_JOINTS[side]
        starts = []
        if q_init is not None:
            starts.append(np.asarray(q_init, dtype=np.float64))
        starts.append(np.asarray(POSTURE_BIAS[side], dtype=np.float64))
        generator = np.random.default_rng(1234 + len(starts))
        for _ in range(3):
            starts.append(np.asarray(POSTURE_BIAS[side], dtype=np.float64)
                          + generator.uniform(-0.35, 0.35, len(names)))
        errors = None
        for start in starts:
            try:
                return self._iterate_arm_ik(
                    side, target_pos, target_rot, start, joint_bounds=joint_bounds)
            except RuntimeError as error:
                errors = str(error)
        raise RuntimeError(f"{side} arm IK did not converge: {errors}")

    def _iterate_arm_ik(self, side, target_pos, target_rot, q_init,
                        joint_bounds=None):
        names = ARM_JOINTS[side]
        joint_bounds = joint_bounds or {}
        lower = np.array([joint_bounds.get(n, self.ranges[n])[0] for n in names])
        upper = np.array([joint_bounds.get(n, self.ranges[n])[1] for n in names])
        q = np.clip(np.asarray(q_init, dtype=np.float64), lower, upper)
        vadr = np.array([self.vadr[n] for n in names])
        jacp = np.zeros((3, self.model.nv))
        jacr = np.zeros((3, self.model.nv))
        body = self.wrist_ids[side]
        best_q, best_cost = q.copy(), float("inf")
        for _ in range(MAX_IK_ITERS):
            for name, value in zip(names, q):
                self.data.qpos[self.qadr[name]] = value
            mujoco.mj_forward(self.model, self.data)
            current_pos = self.data.xpos[body]
            current_rot = self.data.xmat[body].reshape(3, 3)
            pos_error = target_pos - current_pos
            rot_error = axis_angle_from_matrix(target_rot @ current_rot.T)
            pos_norm = np.linalg.norm(pos_error)
            rot_norm = np.linalg.norm(rot_error)
            cost = pos_norm + 0.01 * rot_norm
            if cost < best_cost:
                best_cost, best_q = cost, q.copy()
            if pos_norm < IK_POS_TOL and rot_norm < IK_ROT_TOL:
                self.last_ik_residual = (pos_norm, rot_norm)
                return q
            e = np.concatenate((0.7 * pos_error, 0.35 * rot_error))
            mujoco.mj_jacBody(self.model, self.data, jacp, jacr, body)
            jacobian = np.vstack((jacp[:, vadr], jacr[:, vadr]))
            jjt = jacobian @ jacobian.T
            damping = 0.01 + 0.4 * max(0.0, pos_norm - 0.05)
            dq = jacobian.T @ np.linalg.solve(jjt + (damping ** 2) * np.eye(6), e)
            # Nullspace pull toward the posture bias.
            jacobian_pinv = jacobian.T @ np.linalg.inv(jjt + (damping ** 2) * np.eye(6))
            nullspace = np.eye(7) - jacobian_pinv @ jacobian
            dq += nullspace @ (0.02 * (np.asarray(POSTURE_BIAS[side]) - q))
            step = float(np.max(np.abs(dq)))
            if step > IK_STEP_MAX:
                dq *= IK_STEP_MAX / step
            q = np.clip(q + dq, lower, upper)
        q = best_q
        for name, value in zip(names, q):
            self.data.qpos[self.qadr[name]] = value
        mujoco.mj_forward(self.model, self.data)
        residual = (float(np.linalg.norm(target_pos - self.data.xpos[body])),
                    float(np.linalg.norm(axis_angle_from_matrix(
                        target_rot @ self.data.xmat[body].reshape(3, 3).T))))
        if residual[0] < IK_POS_ACCEPT and residual[1] < IK_ROT_ACCEPT:
            self.last_ik_residual = residual
            return q
        raise RuntimeError(f"{side} arm IK did not converge: residual {residual}")


def rotation_z(angle):
    cos, sin = math.cos(angle), math.sin(angle)
    return np.array([[cos, -sin, 0.0], [sin, cos, 0.0], [0.0, 0.0, 1.0]])


def rotation_x(angle):
    cos, sin = math.cos(angle), math.sin(angle)
    return np.array([[1.0, 0.0, 0.0], [0.0, cos, -sin], [0.0, sin, cos]])


def rotation_y(angle):
    cos, sin = math.cos(angle), math.sin(angle)
    return np.array([[cos, 0.0, sin], [0.0, 1.0, 0.0], [-sin, 0.0, cos]])


def grasp_frames(box_centre, yaw, params: GraspParams):
    """Return per-side SE3 wrist targets for pregrasp/hover/grasp/lift poses.

    Each wrist is placed at a front corner.  The thumb wraps onto the front
    face (towards the robot) while index and middle fingers stay on the two
    opposing side faces.  This gives the box a symmetric clamp without using
    the lower edges as hooks.
    """
    cos, sin = math.cos(yaw), math.sin(yaw)
    long_axis = np.array([-sin, cos, 0.0])   # face normals: +/- long axis
    short_axis = np.array([cos, sin, 0.0])   # along the pressed face
    up = np.array([0.0, 0.0, 1.0])
    yaw_rotation = np.array([[cos, -sin, 0.0], [sin, cos, 0.0], [0.0, 0.0, 1.0]])
    frames = {}
    for side, sign in (("left", 1.0), ("right", -1.0)):
        outward = sign * long_axis          # face normal pointing to this hand
        axes = BASELINE_WRIST_AXES[side]
        rotation = yaw_rotation @ rotation_from_axes(
            np.asarray(axes["x"]), np.asarray(axes["y"]), np.asarray(axes["z"]))
        rotation = rotation @ rotation_x(
            -sign * math.radians(params.palm_tilt_deg))
        approach_axes = APPROACH_WRIST_AXES[side]
        approach_rotation = yaw_rotation @ rotation_from_axes(
            np.asarray(approach_axes["x"]), np.asarray(approach_axes["y"]),
            np.asarray(approach_axes["z"]))
        wrist_base = (box_centre
                      + outward * (0.5 * params.box_dims[1] + params.lateral_beyond_face)
                      + up * params.rise_above_top
                      - short_axis * params.fingertip_reach)
        approach_base = (box_centre
                         + outward * (0.5 * params.box_dims[1]
                                       + params.lateral_beyond_face)
                         + up * max(params.rise_above_top, 0.020)
                         - short_axis * params.approach_reach)
        frames[side] = {
            "rotation": rotation,
            "approach_rotation": approach_rotation,
            "outward": outward,
            # Reach the final palm height while still outside the box, then
            # approach horizontally.  A diagonal descent makes the straight
            # fingers land on the top face before the palms reach the sides.
            "pregrasp": (approach_base - short_axis * 0.08
                         + outward * params.approach_standoff),
            # Advance along the box while still outside its side face.  The
            # final hover-to-grasp motion is purely lateral, perpendicular to
            # that face, so the wrist cannot shove the front edge away.
            "hover": approach_base + outward * params.hover_standoff,
            # Descend here with the palm canted away from the box.  After a
            # horizontal entry at this height, the wrist rotates upright at
            # the side face; the open thumb never crosses the top-front edge.
            "outside_grasp": wrist_base + outward * params.align_standoff,
            "grasp": wrist_base,
            "clamp": wrist_base - outward * params.arm_preload,
            "carry": (wrist_base - outward * params.arm_preload
                      + short_axis * params.front_seat),
            "lift": (wrist_base - outward * params.arm_preload
                     + short_axis * (params.front_seat + params.lift_forward)
                     + up * params.lift_height),
        }
    return frames


def finger_targets(side, scale, params: GraspParams):
    """Mirrored DEX3 corner-grip targets clamped to official joint ranges.

    Joint order is thumb yaw/flex/flex, middle flex/flex, index flex/flex.
    The thumb yaw puts its distal pad on the front panel.  The deliberately
    shallower index and middle curls keep both fingers on the side panel and
    prevent them from becoming hooks under the box.
    """
    if not JOINT_RANGES:
        GraspKinematics()
    mirror = 1.0 if side == "left" else -1.0
    # thumb_0 has the same sign on both DEX3 hands; the remaining flexion
    # joints are mirrored.  Treating the whole vector as mirrored leaves the
    # right thumb on the side face instead of the front panel.
    # Keep both thumb flexion joints close to zero.  Opposition comes from the
    # wrist placement at the front corner rather than a hooked fingertip.
    thumb = np.array([0.0, 0.0, 0.0])
    fingers = np.array([-0.18, -0.12, -0.18, -0.12]) * mirror
    opened = FINGER_OPEN[side] if "FINGER_OPEN" in globals() else np.array(
        [0.0, 0.15, 0.15, -0.15, -0.15, -0.15, -0.15]) * mirror
    # Grip scale changes only the two side fingers.  The thumb always reaches
    # the same nearly straight target.
    values = np.concatenate((thumb,
                             opened[3:] + float(scale) * (fingers - opened[3:])))
    low = np.array([JOINT_RANGES[name][0] for name in HAND_JOINTS[side]])
    high = np.array([JOINT_RANGES[name][1] for name in HAND_JOINTS[side]])
    return np.clip(values, low, high)


FINGER_OPEN = {
    # Index and middle stay straight.  The thumb retracts only while the hands
    # travel laterally past the front corner; the final target above extends
    # it almost straight onto the front panel.
    "left": np.array([0.0, -0.70, 0.0, -0.04, -0.04, -0.04, -0.04]),
    "right": np.array([0.0, 0.70, 0.0, 0.04, 0.04, 0.04, 0.04]),
}
