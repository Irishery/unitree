"""Kinematics, IK and grasp target planning for the single-arm bottle task.

The planner is deliberately free of ROS and of live simulator state: it owns a
private read-only MuJoCo model used only for forward kinematics and Jacobians.
The bottle stands upright on the table; one selected hand approaches from the
side, closes around the body and lifts it.  The robot base is stationary for
the whole sequence.
"""
from dataclasses import dataclass
import math
import os
from pathlib import Path

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
IK_POS_ACCEPT = 60.0e-3
IK_ROT_ACCEPT = 15.0 * math.pi / 180.0
IK_STEP_MAX = 0.12

# Shared joint-range cache filled by GraspKinematics so pure helpers such as
# finger_targets can clamp without loading their own model.
JOINT_RANGES = {}

# Known bottle of the tabletop scene, metres.
BOTTLE_DIAMETER = 0.062
BOTTLE_HEIGHT = 0.235


@dataclass
class GraspParams:
    """Tuned bottle grasp geometry; offsets are in the bottle frame, metres."""

    bottle_diameter: float = BOTTLE_DIAMETER
    bottle_height: float = BOTTLE_HEIGHT
    # Wrist origin sits this far behind the bottle axis.  The straight fingers
    # then reach past the cylinder and curl around its side.
    fingertip_reach: float = 0.160
    # Wrist y is radius + this clearance.  A small inward preload gives the
    # palms a compliant contact before the fingers close.
    lateral_clearance: float = 0.011
    # Wrist height relative to the bottle-body centre.
    grasp_height_offset: float = 0.000
    # Approach is split into a straight move and a small lateral seat.  Keeping
    # the open fingers outside the bottle avoids knocking it over on entry.
    approach_distance: float = 0.120
    hover_distance: float = 0.050
    approach_side_clearance: float = 0.080
    # Finger closure scale; 0.95 leaves the compliant DEX3 pads just short of
    # the hard joint stops while still wrapping the 100 mm body.
    finger_close_scale: float = 0.95
    arm_preload: float = 0.0
    lift_height: float = 0.100
    lift_forward: float = 0.0
    clear_height: float = 0.120
    clear_back: float = 0.050


class GraspKinematics:
    """Read-only kinematic model shared by planning and the scenario node."""

    def __init__(self, model_path=None):
        description = Path(os.environ.get(
            "G1_DESCRIPTION_DIR", "/opt/unitree_ros/robots/g1_description"))
        path = str(model_path or os.environ.get("G1_MUJOCO_MODEL")
                   or description / "g1_29dof_with_dex3_bottle.xml")
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
                self.ranges[name] = (float(self.model.jnt_range[jid, 0]),
                                     float(self.model.jnt_range[jid, 1]))
                JOINT_RANGES[name] = self.ranges[name]
        for name in WAIST_JOINTS:
            jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            if jid >= 0:
                self.joint_ids[name] = jid
                self.qadr[name] = int(self.model.jnt_qposadr[jid])
        self.wrist_ids = {
            side: mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY,
                                    f"{side}_wrist_yaw_link")
            for side in ("left", "right")
        }
        bottle_joint = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_JOINT, "pickup_bottle_free")
        if bottle_joint < 0:
            raise RuntimeError("bottle scene has no pickup_bottle_free joint")
        self.bottle_qadr = int(self.model.jnt_qposadr[bottle_joint])
        self.bottle_dofadr = int(self.model.jnt_dofadr[bottle_joint])
        self.bottle_body = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, "pickup_bottle")
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
                continue
            geoms.append(geom_id)
        return geoms

    def _build_collision_pairs(self):
        def arm_of(body):
            for side in ("left", "right"):
                if any(body.startswith(f"{side}_{part}")
                       for part in ("shoulder", "elbow", "wrist", "hand")):
                    return side
            return None

        left = self._collision_geoms(lambda b: arm_of(b) == "left")
        right = self._collision_geoms(lambda b: arm_of(b) == "right")
        robot_rest = self._collision_geoms(lambda b: arm_of(b) is None and b not in ("world",))
        table = [mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, "table_top")]
        bottle = self._collision_geoms(lambda body: body == "pickup_bottle")
        self.clearance_pairs = []
        for side_geoms, other_geoms, label in (
            (left, right + robot_rest + table + bottle, "left-vs-rest"),
            (right, left + robot_rest + table + bottle, "right-vs-rest"),
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

    def set_bottle(self, centre, yaw=0.0):
        self.data.qpos[self.bottle_qadr:self.bottle_qadr + 3] = centre
        half = 0.5 * float(yaw)
        self.data.qpos[self.bottle_qadr + 3:self.bottle_qadr + 7] = [
            math.cos(half), 0.0, 0.0, math.sin(half)]
        mujoco.mj_forward(self.model, self.data)

    def wrist_pose(self, side):
        body = self.wrist_ids[side]
        return self.data.xpos[body].copy(), self.data.xmat[body].reshape(3, 3).copy()

    def arm_positions(self, side):
        return np.array([self.data.qpos[self.qadr[name]] for name in ARM_JOINTS[side]])

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


def axis_angle_from_matrix(rotation):
    trace = np.clip((np.trace(rotation) - 1.0) * 0.5, -1.0, 1.0)
    angle = math.acos(float(trace))
    if angle < 1e-9:
        return np.zeros(3)
    if angle > math.pi - 1e-6:
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


def rotation_y(angle):
    cos, sin = math.cos(angle), math.sin(angle)
    return np.array([[cos, 0.0, sin], [0.0, 1.0, 0.0], [-sin, 0.0, cos]])


def grasp_frames(bottle_centre, yaw, params: GraspParams):
    """Return per-side SE3 wrist targets for the stationary bottle grasp.

    Both wrists stay at the same height and approach along the table's +X
    direction.  The palms face inward, the straight index/middle fingers close
    around the cylinder, and the thumbs support the near side.
    """
    cos, sin = math.cos(yaw), math.sin(yaw)
    approach = np.array([cos, sin, 0.0])
    lateral = np.array([-sin, cos, 0.0])
    up = np.array([0.0, 0.0, 1.0])
    yaw_rotation = np.array([[cos, -sin, 0.0],
                             [sin, cos, 0.0],
                             [0.0, 0.0, 1.0]])
    frames = {}
    for side, sign in (("left", 1.0), ("right", -1.0)):
        outward = sign * lateral
        wrist_base = (bottle_centre
                      + outward * (0.5 * params.bottle_diameter + params.lateral_clearance)
                      + up * params.grasp_height_offset
                      - approach * params.fingertip_reach)
        frames[side] = {
            "rotation": yaw_rotation,
            "outward": outward,
            "pregrasp": (wrist_base
                         - approach * params.approach_distance
                         + outward * params.approach_side_clearance),
            "hover": (wrist_base
                      - approach * params.hover_distance
                      + outward * params.approach_side_clearance),
            "align": wrist_base + outward * params.approach_side_clearance,
            "grasp": wrist_base,
            "clamp": wrist_base - outward * params.arm_preload,
            "lift": (wrist_base - outward * params.arm_preload
                     + approach * params.lift_forward
                     + up * params.lift_height),
            "clear": (wrist_base - approach * params.clear_back
                      + up * params.clear_height),
        }
    return frames


FINGER_OPEN = {
    "left": np.array([0.0, -0.70, 0.0, -0.04, -0.04, -0.04, -0.04]),
    "right": np.array([0.0, 0.70, 0.0, 0.04, 0.04, 0.04, 0.04]),
}
FINGER_CLOSED = {
    "left": np.array([0.0, 0.70, 1.10, -1.00, -1.30, -1.00, -1.30]),
    "right": np.array([0.0, -0.70, -1.10, 1.00, 1.30, 1.00, 1.30]),
}


def finger_targets(side, scale, params: GraspParams):
    """Interpolated DEX3 closure targets clamped to the official ranges."""
    if not JOINT_RANGES:
        GraspKinematics()
    values = (FINGER_OPEN[side]
              + float(scale) * (FINGER_CLOSED[side] - FINGER_OPEN[side]))
    low = np.array([JOINT_RANGES[name][0] for name in HAND_JOINTS[side]])
    high = np.array([JOINT_RANGES[name][1] for name in HAND_JOINTS[side]])
    return np.clip(values, low, high)
