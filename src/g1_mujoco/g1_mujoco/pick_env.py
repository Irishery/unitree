"""Gymnasium environment for contact-only bimanual G1 DEX3 box pickup."""
from pathlib import Path
import os

import gymnasium as gym
from gymnasium import spaces
import mujoco
import numpy as np

from g1_mujoco.sim import ARMS_AT_SIDES, HAND_JOINTS


ARM_JOINTS = [
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
    "left_elbow_joint", "left_wrist_roll_joint", "left_wrist_pitch_joint", "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
    "right_elbow_joint", "right_wrist_roll_joint", "right_wrist_pitch_joint", "right_wrist_yaw_joint",
]
CONTROL_JOINTS = ARM_JOINTS + HAND_JOINTS["left"] + HAND_JOINTS["right"]

# Reuses the collision-free reference pose from pick_controller.py.  The
# fingers are deliberately open here; closing them is part of the task.
PREGRASP = {
    "left_shoulder_pitch_joint": -0.65, "left_shoulder_roll_joint": 0.55,
    "left_shoulder_yaw_joint": -0.20, "left_elbow_joint": 0.87,
    "left_wrist_roll_joint": 0.0, "left_wrist_pitch_joint": 0.02,
    "left_wrist_yaw_joint": 0.0,
    "right_shoulder_pitch_joint": -0.65, "right_shoulder_roll_joint": -0.55,
    "right_shoulder_yaw_joint": 0.20, "right_elbow_joint": 0.87,
    "right_wrist_roll_joint": 0.0, "right_wrist_pitch_joint": 0.02,
    "right_wrist_yaw_joint": 0.0,
    "left_hand_thumb_0_joint": 0.0, "left_hand_thumb_1_joint": 0.15,
    "left_hand_thumb_2_joint": 0.15, "left_hand_middle_0_joint": -0.15,
    "left_hand_middle_1_joint": -0.15, "left_hand_index_0_joint": -0.15,
    "left_hand_index_1_joint": -0.15,
    "right_hand_thumb_0_joint": 0.0, "right_hand_thumb_1_joint": -0.15,
    "right_hand_thumb_2_joint": -0.15, "right_hand_middle_0_joint": 0.15,
    "right_hand_middle_1_joint": 0.15, "right_hand_index_0_joint": 0.15,
    "right_hand_index_1_joint": 0.15,
}
CURRICULUM_STAGES = ("grasp", "approach", "full")


class G1PickEnv(gym.Env):
    """Learn arm/DEX3 joint targets from box pose and proprioception.

    The observation deliberately uses the box pose as perceived in simulation
    rather than raw renderer pixels.  RGB-D detection is trained and validated
    separately; this keeps early contact learning tractable.
    """

    metadata = {"render_modes": []}

    def __init__(self, model_path=None, max_steps=300, physics_steps=5, stage="grasp"):
        super().__init__()
        if stage not in CURRICULUM_STAGES:
            raise ValueError(f"stage must be one of {CURRICULUM_STAGES}, got {stage!r}")
        description = Path(os.environ.get("G1_DESCRIPTION_DIR", "/opt/unitree_ros/robots/g1_description"))
        self.model = mujoco.MjModel.from_xml_path(str(model_path or description / "g1_29dof_with_dex3_tabletop.xml"))
        self.data = mujoco.MjData(self.model)
        self.max_steps = max_steps
        self.physics_steps = physics_steps
        self.stage = stage
        self.joint_ids = {name: mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name) for name in CONTROL_JOINTS}
        self.actuator_ids = {name: mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, name) for name in CONTROL_JOINTS}
        self.box_joint = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "pickup_box_free")
        self.box_qpos = self.model.jnt_qposadr[self.box_joint]
        self.left_tip = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "left_hand_index_1_link")
        self.right_tip = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "right_hand_index_1_link")
        self.box_body = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "pickup_box")
        self.target = np.zeros(len(CONTROL_JOINTS), dtype=np.float64)
        self.qpos_ids = np.asarray([self.model.jnt_qposadr[self.joint_ids[name]] for name in CONTROL_JOINTS])
        self.qvel_ids = np.asarray([self.model.jnt_dofadr[self.joint_ids[name]] for name in CONTROL_JOINTS])
        self.ctrl_ids = np.asarray([self.actuator_ids[name] for name in CONTROL_JOINTS])
        self.joint_ranges = np.asarray([self.model.jnt_range[self.joint_ids[name]] for name in CONTROL_JOINTS])
        fingers = np.asarray(["_hand_" in name for name in CONTROL_JOINTS])
        self.kp = np.where(fingers, 3.0, 35.0)
        self.kd = np.where(fingers, 0.18, 1.8)
        self.torque_limit = np.where(fingers, 0.9, 18.0)
        self.action_scale = np.full(len(CONTROL_JOINTS), 0.035)
        if stage == "grasp":
            # The first stage only learns contact closure/lift.  It cannot
            # waste exploration on rotating the entire arm away from the box.
            self.action_scale[:len(ARM_JOINTS)] = 0.0
        elif stage == "approach":
            self.action_scale[:len(ARM_JOINTS)] = 0.020
        # Target increments are safer and easier to learn than unbounded torques.
        self.action_space = spaces.Box(-1.0, 1.0, shape=(len(CONTROL_JOINTS),), dtype=np.float32)
        # box xyz + both fingertip relative xyz + joint positions + velocities
        size = 3 + 6 + 2 * len(CONTROL_JOINTS)
        self.observation_space = spaces.Box(-np.inf, np.inf, shape=(size,), dtype=np.float32)
        self.step_count = 0
        self.initial_box_z = 0.800

    def joint_position(self, name):
        return self.data.qpos[self.model.jnt_qposadr[self.joint_ids[name]]]

    def joint_velocity(self, name):
        return self.data.qvel[self.model.jnt_dofadr[self.joint_ids[name]]]

    def observation(self):
        box = self.data.xpos[self.box_body].copy()
        left = self.data.xpos[self.left_tip].copy() - box
        right = self.data.xpos[self.right_tip].copy() - box
        qpos = self.data.qpos[self.qpos_ids]
        qvel = self.data.qvel[self.qvel_ids]
        return np.asarray(np.concatenate((box, left, right, qpos, qvel)), dtype=np.float32)

    def contacts(self):
        count = 0
        for index in range(self.data.ncon):
            contact = self.data.contact[index]
            bodies = [mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY,
                                        self.model.geom_bodyid[geom]) or ""
                      for geom in (contact.geom1, contact.geom2)]
            if "pickup_box" in bodies and any("_hand_" in body for body in bodies):
                count += 1
        return count

    def apply_target(self):
        torque = self.kp * (self.target - self.data.qpos[self.qpos_ids]) - self.kd * self.data.qvel[self.qvel_ids]
        self.data.ctrl[self.ctrl_ids] = np.clip(torque, -self.torque_limit, self.torque_limit)

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        mujoco.mj_resetData(self.model, self.data)
        blend = {"grasp": 1.0, "approach": 0.55, "full": 0.0}[self.stage]
        for name in CONTROL_JOINTS:
            side_pose = ARMS_AT_SIDES.get(name, 0.0)
            value = side_pose * (1.0 - blend) + PREGRASP.get(name, side_pose) * blend
            if name in self.joint_ids:
                self.data.qpos[self.model.jnt_qposadr[self.joint_ids[name]]] = value
        # Increase pose randomisation only after the contact skill exists.
        span = {"grasp": (0.02, 0.03), "approach": (0.06, 0.08), "full": (0.08, 0.12)}[self.stage]
        self.data.qpos[self.box_qpos:self.box_qpos + 3] = [
            self.np_random.uniform(0.50 - span[0], 0.50 + span[0]),
            self.np_random.uniform(-span[1], span[1]), self.initial_box_z,
        ]
        self.data.qpos[self.box_qpos + 3:self.box_qpos + 7] = [1.0, 0.0, 0.0, 0.0]
        mujoco.mj_forward(self.model, self.data)
        self.target = self.data.qpos[self.qpos_ids].copy()
        self.step_count = 0
        return self.observation(), {}

    def step(self, action):
        action = np.asarray(action, dtype=np.float64)
        bounded_action = np.clip(action, -1.0, 1.0)
        self.target = np.clip(self.target + self.action_scale * bounded_action,
                              self.joint_ranges[:, 0], self.joint_ranges[:, 1])
        for _ in range(self.physics_steps):
            self.apply_target()
            mujoco.mj_step(self.model, self.data)
        self.step_count += 1
        box = self.data.xpos[self.box_body]
        fingertip_distance = (np.linalg.norm(self.data.xpos[self.left_tip] - box) +
                              np.linalg.norm(self.data.xpos[self.right_tip] - box))
        contact_count = self.contacts()
        lifted = max(0.0, box[2] - self.initial_box_z)
        # Dense hand-distance reward is the signal that takes the policy from
        # pregrasp to first contact; lift is intentionally dominant only after
        # contact has been discovered.
        reward = -1.5 * fingertip_distance + 0.50 * min(contact_count, 4) + 20.0 * lifted
        if contact_count >= 2:
            reward += 2.0
        reward -= 0.01 * float(np.mean(np.square(bounded_action)))
        reward -= 0.001 * float(np.mean(np.square(self.data.qvel[self.qvel_ids])))
        terminated = bool(lifted > 0.12)
        fallen = bool(box[2] < 0.68)
        if fallen:
            reward -= 3.0
        truncated = self.step_count >= self.max_steps
        return self.observation(), float(reward), terminated or fallen, truncated, {
            "contacts": contact_count, "box_height": float(box[2]), "lifted": float(lifted),
        }

    def close(self):
        pass
