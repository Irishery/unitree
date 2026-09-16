"""ROS 2 adapter for a contact-only MuJoCo G1 DEX3-1 tabletop scene."""
from pathlib import Path
import math
import os
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo, Image, JointState, LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, Float64, Header, Int32
from tf2_ros import TransformBroadcaster


HAND_JOINTS = {
    "left": [
        "left_hand_thumb_0_joint", "left_hand_thumb_1_joint", "left_hand_thumb_2_joint",
        "left_hand_middle_0_joint", "left_hand_middle_1_joint",
        "left_hand_index_0_joint", "left_hand_index_1_joint",
    ],
    "right": [
        "right_hand_thumb_0_joint", "right_hand_thumb_1_joint", "right_hand_thumb_2_joint",
        "right_hand_middle_0_joint", "right_hand_middle_1_joint",
        "right_hand_index_0_joint", "right_hand_index_1_joint",
    ],
}

# Visual "arms at sides" rest pose.  The official MJCF's elbow link has a
# mechanical offset, so elbow=0 projects the forearm forward.  The previous
# vertical arm pose is retained; only a small symmetric shoulder roll moves
# the arms outside the torso and thighs in the visual model.
ARMS_AT_SIDES = {
    "left_shoulder_pitch_joint": 0.0,
    "left_shoulder_roll_joint": 0.25,
    "left_shoulder_yaw_joint": 0.0,
    "left_elbow_joint": 1.50,
    "left_wrist_roll_joint": 0.0,
    "left_wrist_pitch_joint": 0.0,
    "left_wrist_yaw_joint": 0.0,
    "left_hand_thumb_0_joint": 0.0,
    "left_hand_thumb_1_joint": 0.25,
    "left_hand_thumb_2_joint": 0.45,
    "left_hand_middle_0_joint": -0.55,
    "left_hand_middle_1_joint": -0.70,
    "left_hand_index_0_joint": -0.55,
    "left_hand_index_1_joint": -0.70,
    "right_shoulder_pitch_joint": 0.0,
    "right_shoulder_roll_joint": -0.25,
    "right_shoulder_yaw_joint": 0.0,
    "right_elbow_joint": 1.50,
    "right_wrist_roll_joint": 0.0,
    "right_wrist_pitch_joint": 0.0,
    "right_wrist_yaw_joint": 0.0,
    "right_hand_thumb_0_joint": 0.0,
    "right_hand_thumb_1_joint": -0.25,
    "right_hand_thumb_2_joint": -0.45,
    "right_hand_middle_0_joint": 0.55,
    "right_hand_middle_1_joint": 0.70,
    "right_hand_index_0_joint": 0.55,
    "right_hand_index_1_joint": 0.70,
}

ROOM_HALF_EXTENT = 3.0
SCAN_SAMPLES = 360
SCAN_RANGE_MIN = 0.15
SCAN_RANGE_MAX = 30.0
SCAN_PERIOD_STEPS = 33
ODOM_PERIOD_STEPS = 10
CMD_TIMEOUT = 0.5
NAV_SCAN_GEOM_GROUP = 2
MID360_TORSO_OFFSET = np.array([0.0002835, 0.00003, 0.428434], dtype=np.float64)
MID360_VERTICAL_ANGLES = np.deg2rad(np.array([-60.0, -50.0, -40.0, -30.0, -20.0, -10.0, 0.0, 8.0]))
NAV_BASE_COLLISION_RADIUS = 0.30
NAV_BASE_COLLISION_MARGIN = 0.05
NAV_SCAN_AABB_MARGIN = 0.03
WALK_OBSTACLE_CONTYPE = "16"
WALK_OBSTACLE_CONAFFINITY = "32"
WALK_GUARD_CONTYPE = "32"
WALK_GUARD_CONAFFINITY = "16"
WALK_GUARD_GROUP = "3"
WALK_GUARD_RADIUS = 0.30
WALK_GUARD_HALF_HEIGHT = 0.45

# Walking-mode deployment constants for Unitree's pretrained 12-DoF G1
# locomotion policy (models/walk/g1_12dof_motion.pt, from unitree_rl_gym
# deploy/pre_train/g1/motion.pt).  Values reproduce
# deploy/deploy_mujoco/configs/g1.yaml so the policy observes exactly the
# distributions it was trained on.
WALK_LEG_JOINTS = [
    "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
    "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
    "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
    "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
]
WALK_DEFAULT_ANGLES = np.array(
    [-0.1, 0.0, 0.0, 0.3, -0.2, 0.0,
     -0.1, 0.0, 0.0, 0.3, -0.2, 0.0], dtype=np.float32)
WALK_KPS = np.array(
    [100, 100, 100, 150, 40, 40, 100, 100, 100, 150, 40, 40], dtype=np.float32)
WALK_KDS = np.array(
    [2, 2, 2, 4, 2, 2, 2, 2, 2, 4, 2, 2], dtype=np.float32)
WALK_ACTION_SCALE = 0.25
WALK_ANG_VEL_SCALE = 0.25
WALK_DOF_VEL_SCALE = 0.05
WALK_CMD_SCALE = np.array([2.0, 2.0, 0.25], dtype=np.float32)
WALK_GAIT_PERIOD = 0.8
WALK_CONTROL_DECIMATION = 10
WALK_NUM_OBS = 47
WALK_FALL_Z = 0.62
# Nominal parked pelvis height.  The gait leaves the pelvis lower; parking
# restores this so tabletop grasp geometry is the same for every cycle.
WALK_STAND_PELVIS_Z = 0.793
WALK_POLICY_PATH = "/ws/models/walk/g1_12dof_motion.pt"

# Official Unitree G1-29dof velocity policy (unitree_rl_lab / IsaacLab).
# Observation (480) = 5-frame history, term-major, of
#   base_ang_vel(3)*0.2, projected_gravity(3), velocity_commands(3),
#   joint_pos_rel(29), joint_vel_rel(29)*0.05, last_action(29)
# Action (29) = default_joint_pos + 0.25 * output, reordered through joint_ids_map.
UNITREE29_POLICY_PATH = "/ws/models/walk/g1_29dof_velocity.onnx"
UNITREE29_JOINT_IDS_MAP = [0, 6, 12, 1, 7, 13, 2, 8, 14, 3, 9, 15, 22, 4, 10, 16,
                           23, 5, 11, 17, 24, 18, 25, 19, 26, 20, 27, 21, 28]
# Motor index order used by the G1 29-DoF Unitree SDK (also the robot MJCF order).
UNITREE29_MOTOR_JOINTS = [
    "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
    "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
    "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
    "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
    "waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint",
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
    "left_elbow_joint", "left_wrist_roll_joint", "left_wrist_pitch_joint", "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
    "right_elbow_joint", "right_wrist_roll_joint", "right_wrist_pitch_joint", "right_wrist_yaw_joint",
]
UNITREE29_DEFAULT = [-0.1, -0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.3, 0.3, 0.3,
                     0.3, -0.2, -0.2, 0.25, -0.25, 0.0, 0.0, 0.0, 0.0, 0.97, 0.97,
                     0.15, -0.15, 0.0, 0.0, 0.0, 0.0]
UNITREE29_KP = [100.0, 100.0, 100.0, 150.0, 40.0, 40.0, 100.0, 100.0, 100.0, 150.0,
                40.0, 40.0, 200.0, 200.0, 200.0, 40.0, 40.0, 40.0, 40.0, 40.0, 40.0,
                40.0, 40.0, 40.0, 40.0, 40.0, 40.0, 40.0, 40.0]
UNITREE29_KD = [2.0, 2.0, 2.0, 4.0, 2.0, 2.0, 2.0, 2.0, 2.0, 4.0, 2.0, 2.0, 5.0,
                5.0, 5.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0,
                10.0, 10.0, 10.0, 10.0, 10.0]
UNITREE29_ACTION_SCALE = 0.25
UNITREE29_HISTORY = 5
UNITREE29_STEP_DECIMATION = 10   # 0.002 s * 10 = 50 Hz policy rate
UNITREE29_CMD_RANGES = ((-0.5, 1.0), (-0.3, 0.3), (-0.2, 0.2))
ARM_NAME_PARTS = ("shoulder", "elbow", "wrist")
# Local velocity servo around the walking policy.  The policy has a
# standstill drift and a velocity gain error on this 29-DoF body (heavier
# than the 12-DoF training robot); feeding cmd + gain*(cmd - measured)
# holds position at zero command and restores tracking.  The measured
# twist comes from finite differences, so it is low-pass filtered first.
WALK_VEL_SERVO_GAIN = 1.5
WALK_VEL_FILTER_TAU = 0.3
# Walking command guard margin around the physical obstacle AABBs.
WALK_BLOCKER_MARGIN = 0.05
# Position hold while idle: the policy creeps ~5 cm/s at zero command, so
# Nav2's zero cmd would let a "standing" robot wander across the room.
# When the command drops to zero, anchor the pose and feed back a small
# correcting velocity until motion resumes.
WALK_HOLD_XY_GAIN = 0.8
WALK_HOLD_YAW_GAIN = 1.0
WALK_HOLD_XY_CLIP = 0.15
WALK_HOLD_YAW_CLIP = 0.3
# The pretrained locomotion policy is a gait controller: even at zero command
# its phase input keeps asking the legs to perform a dynamic stand/walk cycle.
# For a robot waiting for a Nav2 goal, bypass the policy after a short zero-cmd
# grace period and hold the default leg pose.  Because this repository does not
# include a separate static-balance policy, idle simulation also anchors the
# floating base as a park brake; motion commands release it.
WALK_STAND_CMD_EPS = 1e-3
WALK_STAND_DELAY = 0.35
# When leaving the kinematically parked stance, first let the symmetric
# default leg pose carry the free root, then fade in the gait action.  Starting
# the policy at an arbitrary gait phase caused a 4--6 cm lateral impulse that
# shook a held box downward before the first commanded step.
WALK_RELEASE_SETTLE = 2.0 * WALK_GAIT_PERIOD
# Fade the gait action in over a longer window than the anchored settle so the
# first few steps after the base is released are partial-amplitude and do not
# jolt a held box.  The symmetric (double-support) release phase is set by
# WALK_RELEASE_SETTLE above.
WALK_ACTION_RAMP = 3.0
# Arm hold gains for the pick.  The grasp is established with these compliant
# gains.  Experiments showed that any stiffer arm hold (whole arm, ramped, or
# even shoulder-only from the start) pulls the free box out of its seated
# contacts before the first step, or bumps it during the clamp.  The compliant
# gains are therefore kept throughout the carry.
ARM_GRASP_KP, ARM_GRASP_KD, ARM_GRASP_LIMIT = 80.0, 3.0, 35.0
# Arm-hold integral action (Nm per rad-second) and its windup clamp (Nm).
ARM_INT_KI = 120.0
ARM_INT_LIMIT = 12.0
# When the base parks, the integral arm hold is faded out instead of cleared
# instantly: an abrupt reset jerked the loaded arm and shook the free box out
# of the grip just before the place.
ARM_INT_PARK_TAU = 0.5
# Floating-base arm stabilisation.  During locomotion the pelvis oscillates
# (~1 g vertically) and a positional PD lets the hands swing with it.  A
# world-frame sky-hook damper on each wrist (through the body Jacobian) resists
# that hand motion without a reference pose: the slow intended carry is almost
# untouched while the fast gait oscillation is damped.  Both quantities come
# from the model and the joint/base state, so the scheme transfers to hardware.
ARM_STAB_RAMP = 1.0
ARM_STAB_DAMP_LIN = 40.0
ARM_STAB_DAMP_ANG = 3.0
ARM_STAB_LIMIT = 25.0
LITE_HIDDEN_MESH_GROUP = "4"
LITE_VISIBLE_OVAL_GROUP = "1"
LITE_RGBA_LIGHT = "0.70 0.70 0.70 1"
LITE_RGBA_DARK = "0.20 0.20 0.20 1"
LITE_RGBA_SENSOR = "0.05 0.05 0.05 1"


def yaw_to_quaternion(yaw):
    half = yaw * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


def yaw_to_mujoco_quaternion(yaw):
    half = yaw * 0.5
    return math.cos(half), 0.0, 0.0, math.sin(half)


class G1Mujoco(Node):
    def __init__(self):
        super().__init__("g1_mujoco")
        self.declare_parameter("viewer", True)
        self.declare_parameter("viewer_lite", False)
        self.declare_parameter("tabletop_pick", False)
        self.declare_parameter("walk", False)
        self.declare_parameter("walk_policy", WALK_POLICY_PATH)
        # "gym12" = legacy 12-DoF policy, "unitree29" = official G1-29dof ONNX.
        self.declare_parameter("walk_backend", "gym12")
        self.declare_parameter("walk_policy_onnx", UNITREE29_POLICY_PATH)
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("smoothed_cmd_vel_topic", "/cmd_vel_smoothed")
        self.declare_parameter("box_x", 0.40)
        self.declare_parameter("box_y", 0.0)
        self.declare_parameter("box_yaw", 0.0)
        self.declare_parameter("box_side_rails", False)
        self.tabletop_pick = bool(self.get_parameter("tabletop_pick").value)
        self.walk = bool(self.get_parameter("walk").value)
        self.viewer_lite = bool(self.get_parameter("viewer_lite").value)
        rails = bool(self.get_parameter("box_side_rails").value)
        if self.tabletop_pick:
            if self.walk:
                # Combined mode: locomotion contacts plus the physical tabletop
                # so the robot can walk between the tables while still grasping.
                scene_file = ("g1_29dof_with_dex3_tabletop_rails_walk.xml" if rails
                              else "g1_29dof_with_dex3_tabletop_walk.xml")
            else:
                scene_file = ("g1_29dof_with_dex3_tabletop_rails.xml" if rails
                              else "g1_29dof_with_dex3_tabletop.xml")
        else:
            scene_file = "g1_29dof_with_dex3_nav.xml"
        description = Path(os.environ.get("G1_DESCRIPTION_DIR", "/opt/unitree_ros/robots/g1_description"))
        scene_path = str(description / scene_file)
        temporary_scene_paths = []
        if self.walk and not self.tabletop_pick:
            # The nav walk scene rewrites obstacle contact masks; the combined
            # tabletop+walk scene is generated with the correct masks instead.
            scene_path = self.build_walk_scene(scene_path)
            temporary_scene_paths.append(scene_path)
        if self.viewer_lite:
            scene_path = self.build_lite_scene(scene_path)
            temporary_scene_paths.append(scene_path)
        self.model = mujoco.MjModel.from_xml_path(scene_path)
        if self.tabletop_pick:
            for geom_id in range(self.model.ngeom):
                body = mujoco.mj_id2name(
                    self.model, mujoco.mjtObj.mjOBJ_BODY,
                    self.model.geom_bodyid[geom_id]) or ""
                if "_hand_" in body:
                    self.model.geom_friction[geom_id, 0] = 4.0
        for temporary_scene_path in reversed(temporary_scene_paths):
            try:
                os.unlink(temporary_scene_path)
            except OSError:
                pass
        self.data = mujoco.MjData(self.model)
        self.names = [
            mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, i)
            for i in range(self.model.njnt)
            if self.model.jnt_type[i] in (mujoco.mjtJoint.mjJNT_HINGE, mujoco.mjtJoint.mjJNT_SLIDE)
        ]
        self.names = [name for name in self.names if name]
        self.joint_ids = {name: mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
                          for name in self.names}
        self.actuator_ids = {name: mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
                             for name in self.names}
        self.base_joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, "floating_base_joint")
        self.base_qposadr = None
        self.base_qveladr = None
        self.base_z = 0.793
        if self.base_joint_id >= 0:
            self.base_qposadr = int(self.model.jnt_qposadr[self.base_joint_id])
            self.base_qveladr = int(self.model.jnt_dofadr[self.base_joint_id])
            self.base_z = float(self.data.qpos[self.base_qposadr + 2])
        self.torso_body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "torso_link")
        self.nav_scan_geomgroup = np.zeros(6, dtype=np.uint8)
        self.nav_scan_geomgroup[NAV_SCAN_GEOM_GROUP] = 1
        # Keep the arms out of the navigation envelope.  The official qpos0
        # pose projects both forearms forward (fingertips about 36 cm in
        # front of the torso), so in walk mode the visual robot can enter the
        # table even when Nav2 correctly keeps the base footprint clear.
        # ARMS_AT_SIDES folds the forearms down beside the hips and is used
        # for both kinematic and walking navigation.
        for name, position in ARMS_AT_SIDES.items():
            self.data.qpos[self.model.jnt_qposadr[self.joint_ids[name]]] = position
        self.box_joint_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_JOINT, "pickup_box_free")
        self.box_body_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_BODY, "pickup_box")
        self.box_dofadr = (int(self.model.jnt_dofadr[self.box_joint_id])
                           if self.box_joint_id >= 0 else None)
        self.last_box_log_time = 0.0
        if self.tabletop_pick and self.box_joint_id >= 0:
            qadr = int(self.model.jnt_qposadr[self.box_joint_id])
            yaw = float(self.get_parameter("box_yaw").value)
            box_z = float(self.data.qpos[qadr + 2])
            self.data.qpos[qadr:qadr + 7] = [
                float(self.get_parameter("box_x").value),
                float(self.get_parameter("box_y").value), box_z,
                *yaw_to_mujoco_quaternion(yaw),
            ]
        if self.walk:
            # Start the legs in the policy's default pose so the first PD
            # targets match the physical state.
            for name, position in zip(WALK_LEG_JOINTS, WALK_DEFAULT_ANGLES):
                self.data.qpos[self.model.jnt_qposadr[self.joint_ids[name]]] = position
        mujoco.mj_forward(self.model, self.data)
        self.target = {name: self.qpos(name) for name in self.names}
        if self.walk:
            self.setup_walk()
        self.step_count = 0
        self.base_x = 0.0
        self.base_y = 0.0
        self.base_yaw = 0.0
        self.cmd_vel = Twist()
        # odom_vx/vy are expressed in the base frame (Nav2 twist convention),
        # while the MuJoCo free-joint qvel must be in the world frame.
        self.odom_vx = 0.0
        self.odom_vy = 0.0
        self.odom_wz = 0.0
        self.odom_world_vx = 0.0
        self.odom_world_vy = 0.0
        self.last_cmd_time = self.get_clock().now()
        self.last_final_cmd_time = None
        self.last_smoothed_cmd_time = None
        self.last_nav_collision_log_time = 0.0
        self.nav_blockers = self.build_nav_blockers() if not self.tabletop_pick else []
        self.nav_scan_aabbs = self.build_nav_scan_aabbs() if not self.tabletop_pick else []
        # In walk mode the obstacles have real contacts (see
        # build_walk_scene), so the command guard only needs the physical
        # AABB plus a small margin - the full kinematic inflation would
        # freeze the drifting walking base inside the costmap's inflated
        # zone with no command able to move it back out.
        self.walk_blockers = (
            self.build_nav_blockers(inflate=WALK_BLOCKER_MARGIN)
            if (self.walk and not self.tabletop_pick) else [])
        self.declare_parameter("publish_camera", True)
        self.publisher = self.create_publisher(JointState, "/g1/joint_states", 10)
        # Evaluation-only mirror of the actuator commands: position is the
        # commanded joint target, velocity the measured joint velocity and
        # effort the clamped torque written to ctrl.  It lets an offline trial
        # separate "target moved" from "actuator could not hold the target".
        self.target_pub = self.create_publisher(JointState, "/g1/joint_targets", 10)
        self.contacts_pub = self.create_publisher(Int32, "/g1/mujoco/hand_box_contacts", 10)
        self.grasp_pub = self.create_publisher(Bool, "/g1/mujoco/physical_grasp", 10)
        # Perception only needs the RGB-D stream until a pick is planned.  The
        # controller turns rendering off with this topic so the contact
        # simulation runs at full speed during the (long) manipulation motion.
        self.camera_enabled = True
        self.create_subscription(Bool, "/g1/mujoco/camera_enable",
                                 self.set_camera_enabled, 10)
        self.box_truth_pub = self.create_publisher(
            PoseStamped, "/g1/mujoco/evaluation/box_ground_truth", 10)
        # True floating-base pose for evaluation.  /odom is planar (z=0, yaw
        # only), so an offline renderer cannot reconstruct hand/box geometry
        # during walking from odom alone.
        self.base_truth_pub = self.create_publisher(
            PoseStamped, "/g1/mujoco/evaluation/base_ground_truth", 10)
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.scan_pub = self.create_publisher(LaserScan, "/scan", 10)
        self.mid360_points_pub = self.create_publisher(PointCloud2, "/mid360/points", 10)
        self.color_pub = self.create_publisher(Image, "/camera/camera/color/image_raw", 10)
        self.depth_pub = self.create_publisher(Image, "/camera/camera/depth/image_rect_raw", 10)
        self.color_info_pub = self.create_publisher(CameraInfo, "/camera/camera/color/camera_info", 10)
        self.depth_info_pub = self.create_publisher(CameraInfo, "/camera/camera/depth/camera_info", 10)
        self.points_pub = self.create_publisher(PointCloud2, "/camera/camera/depth/color/points", 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        # In Nav2 bringup the controller writes cmd_vel_nav, velocity_smoother
        # writes /cmd_vel_smoothed, and collision_monitor writes the final
        # /cmd_vel.  Prefer final /cmd_vel so obstacle-stop decisions are not
        # bypassed; keep /cmd_vel_smoothed only as a fallback when the monitor
        # is not running.
        cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        smoothed_cmd_vel_topic = str(self.get_parameter("smoothed_cmd_vel_topic").value)
        self.cmd_vel_sub = self.create_subscription(Twist, cmd_vel_topic, self.set_cmd_vel, 10)
        self.smoothed_cmd_vel_sub = None
        if smoothed_cmd_vel_topic:
            self.smoothed_cmd_vel_sub = self.create_subscription(
                Twist, smoothed_cmd_vel_topic, self.set_smoothed_cmd_vel, 10)
        for name in self.names:
            self.create_subscription(Float64, f"/g1/mujoco/joints/{name}/command",
                                     lambda msg, joint=name: self.set_target(joint, msg.data), 10)
        for side, joints in HAND_JOINTS.items():
            self.create_subscription(JointState, f"/g1/dex3/{side}/command",
                                     lambda msg, target=joints: self.set_hand(target, msg), 10)
        self.viewer = None
        if self.get_parameter("viewer").value:
            from mujoco import viewer as mujoco_viewer
            self.viewer = mujoco_viewer.launch_passive(self.model, self.data)
            self.viewer.opt.geomgroup[int(WALK_GUARD_GROUP)] = False
            if self.viewer_lite:
                # Software-GL laptops render the textured G1 meshes slowly.
                # In lite mode build_lite_scene moves the original robot
                # meshes into a hidden group and adds cheap ovals/capsules
                # instead.  Keep textures enabled so the MuJoCo checker grid
                # on the floor remains visible.
                # NOTE: no local "import mujoco" here - it would shadow the
                # module-level import and crash __init__ with
                # UnboundLocalError at the MjModel load above.
                self.viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_TEXTURE] = True
                self.viewer.opt.geomgroup[int(LITE_HIDDEN_MESH_GROUP)] = False
                self.viewer.opt.geomgroup[int(LITE_VISIBLE_OVAL_GROUP)] = True
                self.viewer.user_scn.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = False
                self.model.vis.quality.shadowsize = 0
        self.renderer = None
        if self.get_parameter("publish_camera").value:
            self.renderer = mujoco.Renderer(self.model, width=640, height=480)
        self.timer = self.create_timer(0.002, self.step)
        if self.walk:
            self.get_logger().info(
                "MuJoCo walk simulation: the legs are driven by the pretrained "
                f"locomotion policy, scene={scene_file}")
        else:
            self.get_logger().info(
                "MuJoCo DEX3 contact simulation: no attach or weld is used for pickup_box; "
                f"scene={scene_file}; Nav2 base motion is published kinematically as "
                "odom -> base_footprint, with base_footprint -> pelvis lifting the visual model")
        if self.nav_blockers:
            blocker_names = ", ".join(blocker["name"] for blocker in self.nav_blockers)
            self.get_logger().info(
                "MuJoCo nav safety guard active: base cannot enter inflated obstacles: "
                f"{blocker_names}")

    def qpos(self, name):
        return float(self.data.qpos[self.model.jnt_qposadr[self.joint_ids[name]]])

    def qvel(self, name):
        return float(self.data.qvel[self.model.jnt_dofadr[self.joint_ids[name]]])

    def set_target(self, name, value):
        if name in self.target and abs(value) < 10.0:
            rng = self.model.jnt_range[self.joint_ids[name]]
            self.target[name] = max(float(rng[0]), min(float(rng[1]), value))

    def set_camera_enabled(self, message):
        self.camera_enabled = bool(message.data)

    def set_hand(self, names, message):
        if len(message.position) != 7:
            self.get_logger().error("DEX3 command must contain 7 ordered positions")
            return
        for name, value in zip(names, message.position):
            self.set_target(name, value)

    def set_cmd_vel(self, message):
        self.cmd_vel = message
        now = self.get_clock().now()
        self.last_cmd_time = now
        self.last_final_cmd_time = now

    def set_smoothed_cmd_vel(self, message):
        if self.last_final_cmd_time is not None:
            final_age = (self.get_clock().now() - self.last_final_cmd_time).nanoseconds * 1e-9
            if final_age < CMD_TIMEOUT:
                return
        self.cmd_vel = message
        now = self.get_clock().now()
        self.last_cmd_time = now
        self.last_smoothed_cmd_time = now

    def build_nav_blockers(self, inflate=None):
        if inflate is None:
            inflate = NAV_BASE_COLLISION_RADIUS + NAV_BASE_COLLISION_MARGIN
        blockers = []
        for geom_id in range(self.model.ngeom):
            if int(self.model.geom_group[geom_id]) != NAV_SCAN_GEOM_GROUP:
                continue
            if self.model.geom_type[geom_id] != mujoco.mjtGeom.mjGEOM_BOX:
                continue
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, geom_id) or f"geom_{geom_id}"
            if not (name == "table_top" or name == "pickup_box_geom" or name.startswith("nav_wall_")):
                continue
            center = self.data.geom_xpos[geom_id].copy()
            size = self.model.geom_size[geom_id].copy()
            blockers.append({
                "name": name,
                "min_x": float(center[0] - size[0] - inflate),
                "max_x": float(center[0] + size[0] + inflate),
                "min_y": float(center[1] - size[1] - inflate),
                "max_y": float(center[1] + size[1] + inflate),
            })
        return blockers

    def build_nav_scan_aabbs(self):
        aabbs = []
        for geom_id in range(self.model.ngeom):
            if int(self.model.geom_group[geom_id]) != NAV_SCAN_GEOM_GROUP:
                continue
            if self.model.geom_type[geom_id] != mujoco.mjtGeom.mjGEOM_BOX:
                continue
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, geom_id) or f"geom_{geom_id}"
            if not (name == "table_top" or name == "pickup_box_geom" or name.startswith("nav_wall_")):
                continue
            center = self.data.geom_xpos[geom_id].copy()
            size = self.model.geom_size[geom_id].copy()
            margin = NAV_SCAN_AABB_MARGIN
            aabbs.append({
                "name": name,
                "min_x": float(center[0] - size[0] - margin),
                "max_x": float(center[0] + size[0] + margin),
                "min_y": float(center[1] - size[1] - margin),
                "max_y": float(center[1] + size[1] + margin),
            })
        return aabbs

    def nav_pose_blocked(self, x, y):
        return self.nav_pose_blocked_in(self.nav_blockers, x, y)

    def nav_pose_blocked_in(self, blockers, x, y):
        for blocker in blockers:
            if blocker["min_x"] <= x <= blocker["max_x"] and blocker["min_y"] <= y <= blocker["max_y"]:
                return blocker["name"]
        return None

    def maybe_log_nav_collision(self, blocker_name):
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        if now_sec - self.last_nav_collision_log_time < 1.0:
            return
        self.last_nav_collision_log_time = now_sec
        self.get_logger().warn(
            "Blocked kinematic base command before entering nav obstacle "
            f"'{blocker_name}'. Pick a goal outside the inflated table/wall costmap.")

    def build_walk_scene(self, source):
        # MuJoCo 3.3.6 bakes geom contact masks at compile time: editing
        # model.geom_contype/conaffinity after MjModel creation has no
        # effect on collision filtering (verified empirically).  Walking
        # needs real robot<->ground contacts, so write a scene variant
        # with the robot geoms enabled and compile that.  The file is
        # placed next to the source so the relative meshdir still works.
        tree = ET.parse(source)
        root = tree.getroot()
        pelvis = root.find(".//body[@name='pelvis']")
        if pelvis is None:
            raise RuntimeError(f"no pelvis body in {source}")
        for geom in pelvis.iter("geom"):
            geom.set("contype", "4")
            geom.set("conaffinity", "8")
        # Nav2 plans for a base footprint, not for the exact swinging arm and
        # hand meshes.  If the table/walls collide with every upper-body geom,
        # the walk policy can get stuck "pushing with its hands" on paths that
        # are otherwise valid for the base.  Keep robot meshes for ground
        # contact only and add one simple base guard for nav obstacles.
        ET.SubElement(pelvis, "geom", {
            "name": "walk_nav_body_guard",
            "type": "cylinder",
            "pos": f"0 0 {-WALK_GUARD_HALF_HEIGHT:.3f}",
            "size": f"{WALK_GUARD_RADIUS:.3f} {WALK_GUARD_HALF_HEIGHT:.3f}",
            "contype": WALK_GUARD_CONTYPE,
            "conaffinity": WALK_GUARD_CONAFFINITY,
            "density": "0",
            "group": WALK_GUARD_GROUP,
            "rgba": "0.1 0.6 1.0 0.18",
        })
        # In the nav scene the walls/table/box are scan-only (masks 0/0):
        # the kinematic base was stopped by the AABB guard, but a walking
        # policy can physically drift straight through them.  Give obstacles
        # a guard-only mask: they stop the base guard, while arms/hands keep
        # their normal ground-contact mask and do not snag on the tabletop.
        world = root.find("worldbody")
        if world is not None:
            for geom in world.iter("geom"):
                name = geom.get("name", "")
                if (name.startswith("nav_wall_") or name == "table_top"
                        or name == "pickup_box_geom"):
                    geom.set("contype", WALK_OBSTACLE_CONTYPE)
                    geom.set("conaffinity", WALK_OBSTACLE_CONAFFINITY)
        destination = source + ".walk.tmp.xml"
        tree.write(destination)
        return destination

    def lite_oval_shape(self, body_name):
        rgba = LITE_RGBA_DARK if any(part in body_name for part in ("pelvis", "head", "ankle_roll")) else LITE_RGBA_LIGHT
        if body_name == "torso_link":
            return ("ellipsoid", None, None, None, "0 0 0.16", "0.13 0.10 0.22", LITE_RGBA_LIGHT)
        if body_name.startswith("pelvis"):
            return ("ellipsoid", None, None, None, "0 0 -0.045", "0.13 0.09 0.08", LITE_RGBA_DARK)
        if body_name.startswith("waist"):
            return ("capsule", "z", 0.055, 0.050, "0 0 0.02", None, LITE_RGBA_LIGHT)
        if "hip_pitch" in body_name or "hip_roll" in body_name:
            return ("capsule", "y", 0.080, 0.050, "0 0 0", None, rgba)
        if "hip_yaw" in body_name:
            return ("capsule", "z", 0.180, 0.050, "0 0 -0.07", None, rgba)
        if "knee" in body_name:
            return ("capsule", "z", 0.240, 0.047, "0 0 -0.12", None, rgba)
        if "ankle_pitch" in body_name:
            return ("capsule", "z", 0.070, 0.034, "0 0 -0.02", None, rgba)
        if "ankle_roll" in body_name:
            return ("ellipsoid", None, None, None, "0.035 0 -0.025", "0.12 0.045 0.025", LITE_RGBA_DARK)
        if "shoulder_pitch" in body_name or "shoulder_roll" in body_name:
            return ("capsule", "y", 0.080, 0.042, "0 0 0", None, rgba)
        if "shoulder_yaw" in body_name:
            return ("capsule", "z", 0.150, 0.040, "0 0 -0.06", None, rgba)
        if "elbow" in body_name:
            return ("capsule", "x", 0.150, 0.038, "0.055 0 0", None, rgba)
        if "wrist" in body_name:
            return ("capsule", "x", 0.070, 0.028, "0.030 0 0", None, rgba)
        if "hand_palm" in body_name:
            return ("ellipsoid", None, None, None, "0.050 0 0", "0.070 0.030 0.040", rgba)
        if "_hand_" in body_name:
            return ("capsule", "x", 0.055, 0.018, "0.025 0 0", None, rgba)
        return ("capsule", "z", 0.060, 0.030, "0 0 0", None, rgba)

    def capsule_fromto(self, axis, length, pos):
        center = [float(value) for value in pos.split()]
        start = center[:]
        end = center[:]
        axis_index = {"x": 0, "y": 1, "z": 2}[axis]
        start[axis_index] -= length * 0.5
        end[axis_index] += length * 0.5
        return " ".join(f"{value:.5f}" for value in start + end)

    def add_lite_oval_geom(self, body):
        body_name = body.get("name", "body")
        kind, axis, length, radius, pos, size, rgba = self.lite_oval_shape(body_name)
        attrs = {
            "name": f"lite_{body_name}_oval",
            "type": kind,
            "contype": "0",
            "conaffinity": "0",
            "density": "0",
            "group": LITE_VISIBLE_OVAL_GROUP,
            "rgba": rgba,
        }
        if kind == "capsule":
            attrs["fromto"] = self.capsule_fromto(axis, length, pos)
            attrs["size"] = f"{radius:.4f}"
        else:
            attrs["pos"] = pos
            attrs["size"] = size
        ET.SubElement(body, "geom", attrs)

    def build_lite_scene(self, source):
        # Keep physics/collision geoms intact, but hide the expensive robot
        # mesh drawables and add a low-poly oval/capsule overlay.  This is
        # deliberately viewer-only: table/walls/lidar scan geometry and Nav2
        # blockers are left untouched.
        tree = ET.parse(source)
        root = tree.getroot()
        world = root.find("worldbody")
        if world is None:
            raise RuntimeError(f"no worldbody in {source}")
        pelvis = world.find("body[@name='pelvis']")
        if pelvis is None:
            raise RuntimeError(f"no pelvis body in {source}")
        for geom in pelvis.iter("geom"):
            if geom.get("type") == "mesh" or geom.get("mesh"):
                geom.set("group", LITE_HIDDEN_MESH_GROUP)
        for body in pelvis.iter("body"):
            self.add_lite_oval_geom(body)
        torso = pelvis.find(".//body[@name='torso_link']")
        if torso is not None:
            ET.SubElement(torso, "geom", {
                "name": "lite_head_oval",
                "type": "ellipsoid",
                "pos": "0.045 0 0.340",
                "size": "0.090 0.075 0.085",
                "contype": "0",
                "conaffinity": "0",
                "density": "0",
                "group": LITE_VISIBLE_OVAL_GROUP,
                "rgba": LITE_RGBA_SENSOR,
            })
        destination = source + ".lite.tmp.xml"
        tree.write(destination)
        return destination

    def setup_walk(self):
        # Runtime reconfiguration toward the policy's training conditions:
        # real gravity (the kinematic scenes compensate it), leg joint
        # damping/armature/frictionloss from the legged_gym defaults.
        self.model.body_gravcomp[:] = 0.0
        joint_ids = [self.joint_ids[name] for name in WALK_LEG_JOINTS]
        self.walk_qadr = np.array([self.model.jnt_qposadr[j] for j in joint_ids])
        # NOTE: for hinges behind the 7-dof free joint, jnt_dofadr ==
        # jnt_qposadr - 1; qpos and qvel addresses are NOT interchangeable.
        self.walk_vadr = np.array([self.model.jnt_dofadr[j] for j in joint_ids])
        self.walk_aid = np.array([self.actuator_ids[name] for name in WALK_LEG_JOINTS])
        self.walk_leg_set = set(WALK_LEG_JOINTS)
        self.model.dof_damping[self.walk_vadr] = 0.001
        self.model.dof_armature[self.walk_vadr] = 0.01
        self.model.dof_frictionloss[self.walk_vadr] = 0.1
        self.walk_backend = str(self.get_parameter("walk_backend").value)
        self.walk_static_stand = True
        self.walk_stand_anchor = None
        self.walk_vx_filt = 0.0
        self.walk_vy_filt = 0.0
        self.walk_wz_filt = 0.0
        self.walk_last_nonzero_cmd_time = None
        self.walk_release_anchored = False
        self.walk_release_start_step = None
        self.walk_release_until_step = None
        self.walk_phase_origin_step = 0
        # Integral action on the arm hold.  During locomotion the torso's
        # vertical acceleration loads the arm joints with a steady inertial
        # torque (~1 g on a ~3 kg arm) that pure PD cannot reject without so
        # much stiffness that the grasp is disturbed.  The integral is held at
        # zero while the grasp is being formed and only accumulates from the
        # walk release preparation, while the base is still parked, so the
        # needed bias is present before the first real step.
        self.arm_integral = {name: 0.0 for name in self.names}
        # Arm chains used by the floating-base stabilisation.
        self.arm_joint_names = [
            name for name in self.names
            if any(part in name for part in ARM_NAME_PARTS)
            and "_hand_" not in name]
        self.arm_dofs_full = np.array(
            [int(self.model.jnt_dofadr[self.joint_ids[name]])
             for name in self.arm_joint_names], dtype=np.int64)
        self.arm_side = {
            name: ("left" if name.startswith("left") else "right")
            for name in self.arm_joint_names}
        self.wrist_body_ids = {
            side: mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY,
                                    f"{side}_wrist_yaw_link")
            for side in ("left", "right")}
        self.base_dofs = (
            np.arange(self.base_qveladr, self.base_qveladr + 6, dtype=np.int64)
            if self.base_qveladr is not None else np.zeros(0, dtype=np.int64))
        self.arm_jacp = np.zeros((3, self.model.nv))
        self.arm_jacr = np.zeros((3, self.model.nv))
        self.arm_stab_ff = {name: 0.0 for name in self.arm_joint_names}
        self.last_fall_log_time = 0.0
        self.prev_torso_attitude = None
        self.last_torso_log_time = 0.0
        if self.walk_backend == "unitree29":
            self.setup_walk_unitree29()
            return
        # torch is only imported when walking is actually requested.
        import torch
        policy_path = str(self.get_parameter("walk_policy").value)
        self.walk_policy = torch.jit.load(policy_path)
        self.walk_policy.eval()
        self.walk_action = np.zeros(len(WALK_LEG_JOINTS), dtype=np.float32)
        self.walk_target = WALK_DEFAULT_ANGLES.copy()
        self.walk_obs = np.zeros(WALK_NUM_OBS, dtype=np.float32)
        self.walk_vx_filt = 0.0
        self.walk_vy_filt = 0.0
        self.walk_wz_filt = 0.0
        self.walk_hold = None
        self.walk_static_stand = True
        self.walk_stand_anchor = None
        self.walk_last_nonzero_cmd_time = None
        self.last_fall_log_time = 0.0
        self.prev_torso_attitude = None
        self.last_torso_log_time = 0.0
        self.get_logger().info(
            "MuJoCo walk mode: Unitree 12-DoF locomotion policy drives the legs "
            f"(pd at 500 Hz, policy at {int(1.0 / (WALK_CONTROL_DECIMATION * self.model.opt.timestep))} Hz); "
            "/cmd_vel is the velocity command and odom is derived from the physics root; "
            "zero cmd parks the simulated root and holds a static standing pose "
            "instead of running the gait policy; release uses a symmetric settle "
            "and action ramp")

    def setup_walk_unitree29(self):
        import onnxruntime as ort
        path = str(self.get_parameter("walk_policy_onnx").value)
        self.u29_session = ort.InferenceSession(path, providers=["CPUExecutionProvider"])
        self.u29_input = self.u29_session.get_inputs()[0].name
        self.u29_policy_joints = [UNITREE29_MOTOR_JOINTS[j] for j in UNITREE29_JOINT_IDS_MAP]
        self.u29_joint_set = set(self.u29_policy_joints)
        self.u29_qadr = np.array([int(self.model.jnt_qposadr[self.joint_ids[n]])
                                  for n in self.u29_policy_joints])
        self.u29_vadr = np.array([int(self.model.jnt_dofadr[self.joint_ids[n]])
                                  for n in self.u29_policy_joints])
        self.u29_aid = np.array([self.actuator_ids[n] for n in self.u29_policy_joints])
        self.u29_default = np.array(UNITREE29_DEFAULT, dtype=np.float64)
        self.u29_kp = np.array(UNITREE29_KP, dtype=np.float64)
        self.u29_kd = np.array(UNITREE29_KD, dtype=np.float64)
        self.u29_arm_mask = np.array([any(p in n for p in ARM_NAME_PARTS)
                                      for n in self.u29_policy_joints])
        count = len(self.u29_policy_joints)
        self.u29_action = np.zeros(count, dtype=np.float32)
        self.u29_last_action = np.zeros(count, dtype=np.float32)
        from collections import deque
        self.u29_hist = {key: deque(maxlen=UNITREE29_HISTORY)
                         for key in ("ang", "grav", "cmd", "jpos", "jvel", "act")}
        self.u29_hist_ready = False
        self.walk_action = np.zeros(len(WALK_LEG_JOINTS), dtype=np.float32)
        self.walk_target = WALK_DEFAULT_ANGLES.copy()
        # Start from the policy's nominal stance so the first observation is a
        # valid standing state.
        for index, name in enumerate(self.u29_policy_joints):
            self.data.qpos[self.u29_qadr[index]] = self.u29_default[index]
        mujoco.mj_forward(self.model, self.data)
        for name in self.names:
            self.target[name] = self.qpos(name)
        # While the command is idle the base is parked (as in the gym12 path) so
        # the manipulation is done from a still, upright stance; it is released
        # as soon as /cmd_vel becomes non-zero and the policy then walks.
        self.walk_static_stand = True
        self.get_logger().info(
            f"MuJoCo walk mode: Unitree G1-29dof velocity ONNX policy {path} "
            "(50 Hz, obs=480, arms overridden by the manipulation controller)")

    def _unitree29_observation(self):
        qw, qx, qy, qz = self.data.qpos[self.base_qposadr + 3:self.base_qposadr + 7]
        rotation = np.array([
            [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
            [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
            [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
        ])
        gravity = rotation.T @ np.array([0.0, 0.0, -1.0])
        # MuJoCo free-joint angular velocity is already in the body frame.
        ang_vel = self.data.qvel[self.base_qveladr + 3:self.base_qveladr + 6]
        vx, vy, wz = self.raw_walk_cmd()
        ranges = UNITREE29_CMD_RANGES
        command = np.array([
            min(max(vx, ranges[0][0]), ranges[0][1]),
            min(max(vy, ranges[1][0]), ranges[1][1]),
            min(max(wz, ranges[2][0]), ranges[2][1]),
        ])
        terms = {
            "ang": np.asarray(ang_vel, float) * 0.2,
            "grav": np.asarray(gravity, float),
            "cmd": command.astype(float),
            "jpos": np.asarray(self.data.qpos[self.u29_qadr] - self.u29_default, float),
            "jvel": np.asarray(self.data.qvel[self.u29_vadr], float) * 0.05,
            "act": self.u29_last_action.astype(float),
        }
        for key, value in terms.items():
            self.u29_hist[key].append(value.astype(np.float32))
        if not self.u29_hist_ready:
            for key in self.u29_hist:
                while len(self.u29_hist[key]) < UNITREE29_HISTORY:
                    self.u29_hist[key].appendleft(self.u29_hist[key][0])
            self.u29_hist_ready = True
        observation = []
        for key in ("ang", "grav", "cmd", "jpos", "jvel", "act"):
            for frame in self.u29_hist[key]:
                observation.extend(frame)
        return np.asarray(observation, dtype=np.float32)[None, :]

    def infer_unitree29(self):
        observation = self._unitree29_observation()
        out = self.u29_session.run(None, {self.u29_input: observation})[0][0]
        self.u29_last_action = out.astype(np.float32)
        self.u29_action = out

    def control_walk_unitree29(self):
        if self.walk_should_static_stand():
            self.static_stand_walk()
            self.apply_static_walk_root()
            targets = self.u29_default
        else:
            self.walk_static_stand = False
            self.walk_stand_anchor = None
            if self.step_count % UNITREE29_STEP_DECIMATION == 0:
                self.infer_unitree29()
            targets = self.u29_default + UNITREE29_ACTION_SCALE * self.u29_action
        if self.step_count % 250 == 0:
            self.get_logger().info(
                f"u29 pos=({self.data.qpos[self.base_qposadr]:.3f},"
                f"{self.data.qpos[self.base_qposadr + 1]:.3f}) "
                f"base_z={self.data.qpos[self.base_qposadr + 2]:.3f} "
                f"static={self.walk_static_stand}")
        for index, name in enumerate(self.u29_policy_joints):
            position = float(self.data.qpos[self.u29_qadr[index]])
            velocity = float(self.data.qvel[self.u29_vadr[index]])
            if self.u29_arm_mask[index] and self.tabletop_pick:
                # During manipulation the arms stay under the pick controller.
                wanted, kp, kd, limit = self.target[name], 80.0, 3.0, 35.0
            else:
                wanted = float(targets[index])
                kp, kd, limit = self.u29_kp[index], self.u29_kd[index], 60.0
            torque = kp * (wanted - position) - kd * velocity
            self.data.ctrl[self.u29_aid[index]] = max(-limit, min(limit, torque))
        for name, actuator in self.actuator_ids.items():
            if name in self.u29_joint_set:
                continue
            finger = "_hand_" in name
            kp, kd = (12.0, 0.45) if finger else (80.0, 3.0)
            limit = 3.0 if finger else 35.0
            torque = kp * (self.target[name] - self.qpos(name)) - kd * self.qvel(name)
            self.data.ctrl[actuator] = max(-limit, min(limit, torque))

    def stabilize_torso(self):
        """IMU-style torso attitude hold using the waist joints.

        Keeps the chest level while the legs walk, so a carried object is not
        shaken by the gait.  This uses only the measured torso orientation and
        the waist actuators, so the same loop transfers to the real G1.
        """
        if self.torso_body_id < 0:
            return
        rotation = self.data.xmat[self.torso_body_id].reshape(3, 3)
        pitch = math.atan2(-rotation[2, 0],
                           math.hypot(rotation[2, 1], rotation[2, 2]))
        roll = math.atan2(rotation[2, 1], rotation[2, 2])
        dt = float(self.model.opt.timestep)
        if self.prev_torso_attitude is None:
            pitch_rate = roll_rate = 0.0
        else:
            pitch_rate = (pitch - self.prev_torso_attitude[0]) / dt
            roll_rate = (roll - self.prev_torso_attitude[1]) / dt
        self.prev_torso_attitude = (pitch, roll)
        kp, kd = 120.0, 12.0
        for name, error, rate in (("waist_pitch_joint", pitch, pitch_rate),
                                  ("waist_roll_joint", roll, roll_rate)):
            low, high = self.model.jnt_range[self.joint_ids[name]]
            command = -kp * error - kd * rate
            self.target[name] = max(float(low), min(float(high), command))
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        if now_sec - self.last_torso_log_time > 1.0:
            self.last_torso_log_time = now_sec
            self.get_logger().info(
                f"torso hold: pitch={math.degrees(pitch):.1f}deg roll={math.degrees(roll):.1f}deg")

    def stabilize_arm_motion(self):
        """World-frame damping that keeps the hands off the gait oscillation.

        Uses the wrist body Jacobian to add a sky-hook damper on the hand's
        world velocity.  It needs no reference pose and is ~0 at rest, so it is
        faded in from the walk release preparation and cannot disturb the grasp.
        """
        if not self.tabletop_pick or self.base_dofs.size == 0:
            return
        if self.walk_static_stand and self.walk_release_start_step is None:
            for name in self.arm_joint_names:
                self.arm_stab_ff[name] = 0.0
            return
        start = self.walk_release_start_step
        if start is None:
            alpha = 1.0
        else:
            elapsed = (self.step_count - start) * float(self.model.opt.timestep)
            alpha = min(1.0, max(0.0, elapsed / ARM_STAB_RAMP))
        for side, body_id in self.wrist_body_ids.items():
            if body_id < 0:
                continue
            mujoco.mj_jacBody(self.model, self.data, self.arm_jacp,
                              self.arm_jacr, body_id)
            linear = self.arm_jacp @ self.data.qvel
            angular = self.arm_jacr @ self.data.qvel
            force = -ARM_STAB_DAMP_LIN * linear
            moment = -ARM_STAB_DAMP_ANG * angular
            tau = self.arm_jacp.T @ force + self.arm_jacr.T @ moment
            for index, name in enumerate(self.arm_joint_names):
                if self.arm_side[name] != side:
                    continue
                value = alpha * float(tau[self.arm_dofs_full[index]])
                self.arm_stab_ff[name] = max(-ARM_STAB_LIMIT,
                                             min(ARM_STAB_LIMIT, value))

    def control_walk(self):
        # Level the chest while the gait runs.  While parked the waist keeps
        # its pose; actively re-leveling it during the grasp changed how the
        # free box seated on the thumbs and let it hang on the rails.
        if self.tabletop_pick and not self.walk_static_stand:
            self.stabilize_torso()
        self.stabilize_arm_motion()
        if self.step_count % 500 == 0:
            cmd = self.walk_cmd()
            self.get_logger().info(
                f"walk base=({self.base_x:.3f},{self.base_y:.3f}) yaw={self.base_yaw:.2f} "
                f"servo_cmd=({cmd[0]:.3f},{cmd[1]:.3f},{cmd[2]:.3f}) "
                f"meas=({self.walk_vx_filt:.3f},{self.walk_vy_filt:.3f}) "
                f"static={self.walk_static_stand}")
        # Policy PD on the legs.
        q_leg = self.data.qpos[self.walk_qadr]
        v_leg = self.data.qvel[self.walk_vadr]
        self.data.ctrl[self.walk_aid] = (self.walk_target - q_leg) * WALK_KPS - v_leg * WALK_KDS
        # Stiff PD on everything else so the arms/waist behave like the
        # rigid torso the policy was trained with (soft gains let the
        # upper body swing and destabilise the gait).
        for name, actuator in self.actuator_ids.items():
            if name in self.walk_leg_set:
                continue
            finger = "_hand_" in name
            if self.tabletop_pick:
                # Keep the validated compliant tabletop gains in the combined
                # walk scene as well.  Stiffer arm holds were tried and pulled
                # the free box out of its seated contacts at gait release.
                arm = (not finger) and any(
                    part in name for part in ARM_NAME_PARTS)
                kp, kd = (12.0, 0.45) if finger else (ARM_GRASP_KP, ARM_GRASP_KD)
                limit = 3.0 if finger else ARM_GRASP_LIMIT
                error = self.target[name] - self.qpos(name)
                integral = 0.0
                if arm:
                    if self.walk_static_stand and self.walk_release_start_step is None:
                        self.arm_integral[name] *= math.exp(
                            -float(self.model.opt.timestep) / ARM_INT_PARK_TAU)
                    else:
                        self.arm_integral[name] = max(
                            -ARM_INT_LIMIT, min(ARM_INT_LIMIT,
                                                self.arm_integral[name]
                                                + ARM_INT_KI * error
                                                * float(self.model.opt.timestep)))
                    integral = self.arm_integral[name]
                torque = (kp * error + integral + self.arm_stab_ff.get(name, 0.0)
                          - kd * self.qvel(name))
            else:
                kp, kd = (3.0, 0.18) if finger else (150.0, 4.0)
                limit = 0.9 if finger else 60.0
                torque = kp * (self.target[name] - self.qpos(name)) - kd * self.qvel(name)
            self.data.ctrl[actuator] = max(-limit, min(limit, torque))

    def raw_walk_cmd(self):
        age = (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9
        if age > CMD_TIMEOUT:
            return 0.0, 0.0, 0.0
        return (
            max(-0.35, min(0.35, float(self.cmd_vel.linear.x))),
            max(-0.25, min(0.25, float(self.cmd_vel.linear.y))),
            max(-0.8, min(0.8, float(self.cmd_vel.angular.z))),
        )

    def walk_cmd_is_idle(self, vx, vy, wz):
        return (
            abs(vx) < WALK_STAND_CMD_EPS
            and abs(vy) < WALK_STAND_CMD_EPS
            and abs(wz) < WALK_STAND_CMD_EPS
        )

    def walk_should_static_stand(self):
        vx, vy, wz = self.raw_walk_cmd()
        now = self.get_clock().now()
        if not self.walk_cmd_is_idle(vx, vy, wz):
            self.walk_last_nonzero_cmd_time = now
            return False
        if self.walk_last_nonzero_cmd_time is None:
            return True
        idle_age = (now - self.walk_last_nonzero_cmd_time).nanoseconds * 1e-9
        return idle_age >= WALK_STAND_DELAY

    def walk_cmd(self):
        vx, vy, wz = self.raw_walk_cmd()
        # Walking: real contacts stop the base at the physical obstacles,
        # so the guard only zeroes commands right at the physical AABBs.
        blocker_name = self.nav_pose_blocked_in(
            self.walk_blockers if self.walk else self.nav_blockers,
            self.base_x, self.base_y)
        if blocker_name:
            self.maybe_log_nav_collision(blocker_name)
            return np.zeros(3, dtype=np.float32)
        # Velocity servo: correct the policy's standstill drift and gain
        # error against the filtered measured body twist.
        if self.walk_cmd_is_idle(vx, vy, wz):
            # Idle: anchor and hold the pose against the policy's creep.
            if self.walk_hold is None:
                self.walk_hold = (self.base_x, self.base_y, self.base_yaw)
            hx, hy, hyaw = self.walk_hold
            ex, ey = hx - self.base_x, hy - self.base_y
            cos_yaw = math.cos(self.base_yaw)
            sin_yaw = math.sin(self.base_yaw)
            body_x = ex * cos_yaw + ey * sin_yaw
            body_y = -ex * sin_yaw + ey * cos_yaw
            eyaw = math.atan2(math.sin(hyaw - self.base_yaw), math.cos(hyaw - self.base_yaw))
            vx = max(-WALK_HOLD_XY_CLIP, min(WALK_HOLD_XY_CLIP, WALK_HOLD_XY_GAIN * body_x))
            vy = max(-WALK_HOLD_XY_CLIP, min(WALK_HOLD_XY_CLIP, WALK_HOLD_XY_GAIN * body_y))
            wz = max(-WALK_HOLD_YAW_CLIP, min(WALK_HOLD_YAW_CLIP, WALK_HOLD_YAW_GAIN * eyaw))
        else:
            self.walk_hold = None
            vx += WALK_VEL_SERVO_GAIN * (vx - self.walk_vx_filt)
            vy += WALK_VEL_SERVO_GAIN * (vy - self.walk_vy_filt)
            wz += WALK_VEL_SERVO_GAIN * (wz - self.walk_wz_filt)
        return np.array([
            max(-1.0, min(1.0, vx)),
            max(-1.0, min(1.0, vy)),
            max(-1.0, min(1.0, wz)),
        ], dtype=np.float32)

    def infer_walk_policy(self):
        obs = self.walk_obs
        q_leg = self.data.qpos[self.walk_qadr]
        v_leg = self.data.qvel[self.walk_vadr]
        obs[:3] = self.data.qvel[3:6] * WALK_ANG_VEL_SCALE
        qw, qx, qy, qz = self.data.qpos[3:7]
        obs[3] = 2.0 * (-qz * qx + qw * qy)
        obs[4] = -2.0 * (qz * qy + qw * qx)
        obs[5] = 1.0 - 2.0 * (qw * qw + qz * qz)
        obs[6:9] = self.walk_cmd() * WALK_CMD_SCALE
        obs[9:21] = q_leg - WALK_DEFAULT_ANGLES
        obs[21:33] = v_leg * WALK_DOF_VEL_SCALE
        obs[33:45] = self.walk_action
        phase_step = max(0, self.step_count - self.walk_phase_origin_step)
        phase = ((phase_step * self.model.opt.timestep) % WALK_GAIT_PERIOD) / WALK_GAIT_PERIOD
        obs[45] = math.sin(2.0 * math.pi * phase)
        obs[46] = math.cos(2.0 * math.pi * phase)
        import torch
        with torch.no_grad():
            action = self.walk_policy(
                torch.from_numpy(obs).unsqueeze(0)).detach().numpy().squeeze()
        if self.walk_release_start_step is not None:
            ramp_steps = max(1, int(WALK_ACTION_RAMP / self.model.opt.timestep))
            alpha = min(1.0, max(
                0.0, (self.step_count - self.walk_release_start_step) / ramp_steps))
            action *= alpha
        self.walk_action = action
        self.walk_target = self.walk_action * WALK_ACTION_SCALE + WALK_DEFAULT_ANGLES

    def update_walk_odom(self):
        qpos = self.data.qpos
        x, y = float(qpos[0]), float(qpos[1])
        qw, qx, qy, qz = qpos[3:7]
        yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
        dt = float(self.model.opt.timestep)
        dx, dy = x - self.base_x, y - self.base_y
        cos_yaw = math.cos(self.base_yaw)
        sin_yaw = math.sin(self.base_yaw)
        self.odom_vx = (dx * cos_yaw + dy * sin_yaw) / dt
        self.odom_vy = (-dx * sin_yaw + dy * cos_yaw) / dt
        dyaw = math.atan2(math.sin(yaw - self.base_yaw), math.cos(yaw - self.base_yaw))
        self.odom_wz = dyaw / dt
        self.base_x, self.base_y, self.base_yaw = x, y, yaw
        self.base_z = float(qpos[2])
        # Low-pass the finite-difference twist for the velocity servo.
        alpha = dt / (dt + WALK_VEL_FILTER_TAU)
        self.walk_vx_filt += alpha * (self.odom_vx - self.walk_vx_filt)
        self.walk_vy_filt += alpha * (self.odom_vy - self.walk_vy_filt)
        self.walk_wz_filt += alpha * (self.odom_wz - self.walk_wz_filt)
        if qpos[2] < WALK_FALL_Z:
            now_sec = self.get_clock().now().nanoseconds * 1e-9
            if now_sec - self.last_fall_log_time > 1.0:
                self.last_fall_log_time = now_sec
                self.get_logger().error(
                    f"Walking policy fell: pelvis z={qpos[2]:.2f} below {WALK_FALL_Z}")

    def static_stand_walk(self):
        if not self.walk_static_stand or self.walk_stand_anchor is None:
            self.walk_hold = (self.base_x, self.base_y, self.base_yaw)
            # Park at the nominal standing height, not the (often crouched)
            # height left by the gait: a low pelvis makes a tabletop box appear
            # high and out of the arm workspace for the next pick.
            self.walk_stand_anchor = (
                self.base_x, self.base_y, self.base_yaw, WALK_STAND_PELVIS_Z)
        self.walk_static_stand = True
        self.walk_action[:] = 0.0
        self.walk_target = WALK_DEFAULT_ANGLES.copy()

    def apply_static_walk_root(self):
        if self.base_qposadr is None or self.base_qveladr is None:
            return
        if self.walk_stand_anchor is None:
            self.walk_stand_anchor = (
                self.base_x, self.base_y, self.base_yaw, WALK_STAND_PELVIS_Z)
        x, y, yaw, z = self.walk_stand_anchor
        qpos = self.data.qpos
        qvel = self.data.qvel
        qpos[self.base_qposadr + 0] = x
        qpos[self.base_qposadr + 1] = y
        qpos[self.base_qposadr + 2] = z
        qw, qx, qy, qz = yaw_to_mujoco_quaternion(yaw)
        qpos[self.base_qposadr + 3] = qw
        qpos[self.base_qposadr + 4] = qx
        qpos[self.base_qposadr + 5] = qy
        qpos[self.base_qposadr + 6] = qz
        qvel[self.base_qveladr:self.base_qveladr + 6] = 0.0

    def step(self):
        if self.walk:
            if self.walk_backend == "unitree29":
                self.control_walk_unitree29()
            else:
                self.control_walk()
                if self.walk_static_stand:
                    if not self.walk_release_anchored:
                        self.static_stand_walk()
                    self.apply_static_walk_root()
        else:
            self.integrate_base()
            self.apply_base_pose()
            for name, actuator in self.actuator_ids.items():
                finger = "_hand_" in name
                if self.tabletop_pick:
                    kp, kd = (12.0, 0.45) if finger else (80.0, 3.0)
                    limit = 3.0 if finger else 35.0
                else:
                    kp, kd = (3.0, 0.18) if finger else (35.0, 1.8)
                    limit = 0.9 if finger else 18.0
                torque = kp * (self.target[name] - self.qpos(name)) - kd * self.qvel(name)
                self.data.ctrl[actuator] = max(-limit, min(limit, torque))
        mujoco.mj_step(self.model, self.data)
        if self.walk:
            if self.walk_static_stand:
                self.apply_static_walk_root()
            self.update_walk_odom()
        else:
            self.apply_base_pose()
        mujoco.mj_forward(self.model, self.data)
        self.step_count += 1
        if self.tabletop_pick and self.box_dofadr is not None:
            speed = float(np.linalg.norm(
                self.data.qvel[self.box_dofadr:self.box_dofadr + 3]))
            if speed > 2.0:
                now_sec = self.get_clock().now().nanoseconds * 1e-9
                if now_sec - self.last_box_log_time > 0.5:
                    self.last_box_log_time = now_sec
                    pairs = []
                    for index in range(self.data.ncon):
                        contact = self.data.contact[index]
                        bodies = [mujoco.mj_id2name(
                            self.model, mujoco.mjtObj.mjOBJ_BODY,
                            self.model.geom_bodyid[geom]) or ""
                            for geom in (contact.geom1, contact.geom2)]
                        if "pickup_box" in bodies:
                            pairs.append((bodies[0], bodies[1], round(float(contact.dist), 4)))
                    self.get_logger().warn(
                        f"box speed {speed:.1f} m/s at {self.data.xpos[self.box_body_id].round(3).tolist()} "
                        f"base=({self.base_x:.3f},{self.base_y:.3f},{self.base_yaw:.2f}) contacts={pairs[:8]}")
        if (self.walk and self.walk_backend != "unitree29"
                and self.step_count % WALK_CONTROL_DECIMATION == 0):
            if self.walk_should_static_stand():
                self.walk_release_anchored = False
                self.walk_release_start_step = None
                self.walk_release_until_step = None
                self.static_stand_walk()
            else:
                if self.walk_static_stand and not self.walk_release_anchored:
                    self.walk_release_anchored = True
                    settle_steps = int(WALK_RELEASE_SETTLE / self.model.opt.timestep)
                    self.walk_release_start_step = self.step_count
                    self.walk_release_until_step = self.step_count + settle_steps
                    self.walk_phase_origin_step = self.step_count
                    self.walk_action[:] = 0.0
                    self.walk_target = WALK_DEFAULT_ANGLES.copy()
                    self.infer_walk_policy()
                elif self.walk_release_anchored:
                    self.infer_walk_policy()
                    if self.step_count >= self.walk_release_until_step:
                        self.walk_static_stand = False
                        self.walk_stand_anchor = None
                        self.walk_release_anchored = False
                        self.walk_release_start_step = None
                        self.walk_release_until_step = None
                else:
                    self.infer_walk_policy()
        if self.step_count % ODOM_PERIOD_STEPS == 0:
            stamp = self.get_clock().now().to_msg()
            self.publish_odom(stamp)
            self.publish_state(stamp)
        if self.step_count % SCAN_PERIOD_STEPS == 0:
            # Publish fresh matching odom -> base_footprint -> pelvis transforms and joint
            # state immediately before the scan.  SLAM Toolbox and Nav2
            # message filters require the scan timestamp to be transformable
            # through both dynamic TF segments; separate now() calls can put
            # scan messages just outside the available TF cache during startup.
            stamp = self.get_clock().now().to_msg()
            self.publish_odom(stamp)
            self.publish_state(stamp)
            self.publish_scan(stamp)
            self.publish_mid360_points(stamp)
        if self.renderer is not None and self.camera_enabled and self.step_count % 17 == 0:
            self.publish_camera()
        if self.viewer is not None and self.step_count % 10 == 0:
            self.viewer.sync()

    def integrate_base(self):
        age = (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9
        if age > CMD_TIMEOUT:
            vx = vy = wz = 0.0
        else:
            vx = max(-0.35, min(0.35, float(self.cmd_vel.linear.x)))
            vy = max(-0.25, min(0.25, float(self.cmd_vel.linear.y)))
            wz = max(-0.8, min(0.8, float(self.cmd_vel.angular.z)))
        dt = float(self.model.opt.timestep)
        cos_yaw = math.cos(self.base_yaw)
        sin_yaw = math.sin(self.base_yaw)
        old_x = self.base_x
        old_y = self.base_y
        dx = (vx * cos_yaw - vy * sin_yaw) * dt
        dy = (vx * sin_yaw + vy * cos_yaw) * dt
        new_x = self.base_x + dx
        new_y = self.base_y + dy

        blocker_name = self.nav_pose_blocked(new_x, new_y)
        if blocker_name:
            slide_x_blocker = self.nav_pose_blocked(self.base_x + dx, self.base_y)
            slide_y_blocker = self.nav_pose_blocked(self.base_x, self.base_y + dy)
            if not slide_x_blocker:
                self.base_x += dx
            elif not slide_y_blocker:
                self.base_y += dy
            else:
                self.maybe_log_nav_collision(blocker_name)
        else:
            self.base_x = new_x
            self.base_y = new_y

        self.base_yaw = math.atan2(
            math.sin(self.base_yaw + wz * dt), math.cos(self.base_yaw + wz * dt))
        actual_dx = self.base_x - old_x
        actual_dy = self.base_y - old_y
        self.odom_vx = (actual_dx * cos_yaw + actual_dy * sin_yaw) / dt
        self.odom_vy = (-actual_dx * sin_yaw + actual_dy * cos_yaw) / dt
        self.odom_world_vx = actual_dx / dt
        self.odom_world_vy = actual_dy / dt
        self.odom_wz = wz

    def apply_base_pose(self):
        if self.base_qposadr is None or self.base_qveladr is None:
            return
        qpos = self.data.qpos
        qvel = self.data.qvel
        qpos[self.base_qposadr + 0] = self.base_x
        qpos[self.base_qposadr + 1] = self.base_y
        qpos[self.base_qposadr + 2] = self.base_z
        qw, qx, qy, qz = yaw_to_mujoco_quaternion(self.base_yaw)
        qpos[self.base_qposadr + 3] = qw
        qpos[self.base_qposadr + 4] = qx
        qpos[self.base_qposadr + 5] = qy
        qpos[self.base_qposadr + 6] = qz
        qvel[self.base_qveladr + 0] = self.odom_world_vx
        qvel[self.base_qveladr + 1] = self.odom_world_vy
        qvel[self.base_qveladr + 2] = 0.0
        qvel[self.base_qveladr + 3] = 0.0
        qvel[self.base_qveladr + 4] = 0.0
        qvel[self.base_qveladr + 5] = self.odom_wz

    def publish_odom(self, stamp=None):
        if stamp is None:
            stamp = self.get_clock().now().to_msg()
        qx, qy, qz, qw = yaw_to_quaternion(self.base_yaw)
        base_transform = TransformStamped()
        base_transform.header.stamp = stamp
        base_transform.header.frame_id = "odom"
        base_transform.child_frame_id = "base_footprint"
        base_transform.transform.translation.x = self.base_x
        base_transform.transform.translation.y = self.base_y
        base_transform.transform.translation.z = 0.0
        base_transform.transform.rotation.x = qx
        base_transform.transform.rotation.y = qy
        base_transform.transform.rotation.z = qz
        base_transform.transform.rotation.w = qw

        pelvis_transform = TransformStamped()
        pelvis_transform.header.stamp = stamp
        pelvis_transform.header.frame_id = "base_footprint"
        pelvis_transform.child_frame_id = "pelvis"
        pelvis_transform.transform.translation.x = 0.0
        pelvis_transform.transform.translation.y = 0.0
        pelvis_transform.transform.translation.z = self.base_z
        pelvis_transform.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform([base_transform, pelvis_transform])

        odom = Odometry()
        odom.header = base_transform.header
        odom.child_frame_id = "base_footprint"
        odom.pose.pose.position.x = self.base_x
        odom.pose.pose.position.y = self.base_y
        odom.pose.pose.orientation = base_transform.transform.rotation
        odom.twist.twist.linear.x = self.odom_vx
        odom.twist.twist.linear.y = self.odom_vy
        odom.twist.twist.angular.z = self.odom_wz
        self.odom_pub.publish(odom)

    def scan_origin(self):
        if self.torso_body_id >= 0:
            rotation = self.data.xmat[self.torso_body_id].reshape(3, 3)
            return self.data.xpos[self.torso_body_id].copy() + rotation @ MID360_TORSO_OFFSET
        return np.array([self.base_x, self.base_y, self.base_z + 0.47], dtype=np.float64)

    def scan_rotation(self):
        if self.torso_body_id >= 0:
            return self.data.xmat[self.torso_body_id].reshape(3, 3).copy()
        cos_yaw = math.cos(self.base_yaw)
        sin_yaw = math.sin(self.base_yaw)
        return np.array([
            [cos_yaw, -sin_yaw, 0.0],
            [sin_yaw, cos_yaw, 0.0],
            [0.0, 0.0, 1.0],
        ], dtype=np.float64)

    def ray_scene_distance(self, origin, ray):
        geomid = np.array([-1], dtype=np.int32)
        distance = mujoco.mj_ray(
            self.model, self.data, origin, ray, self.nav_scan_geomgroup, 1, -1, geomid)
        if distance < 0.0:
            return float("inf")
        return float(distance)

    def ray_scene_projection(self, origin, yaw):
        best = float("inf")
        for pitch in MID360_VERTICAL_ANGLES:
            cos_pitch = math.cos(float(pitch))
            ray = np.array([
                cos_pitch * math.cos(yaw),
                cos_pitch * math.sin(yaw),
                math.sin(float(pitch)),
            ], dtype=np.float64)
            distance = self.ray_scene_distance(origin, ray)
            if not math.isfinite(distance):
                continue
            horizontal_distance = float(distance * cos_pitch)
            if SCAN_RANGE_MIN <= horizontal_distance <= SCAN_RANGE_MAX:
                best = min(best, horizontal_distance)
        return best

    def ray_aabb_projection(self, origin, yaw):
        if not self.nav_scan_aabbs:
            return float("inf")
        ox = float(origin[0])
        oy = float(origin[1])
        dx = math.cos(yaw)
        dy = math.sin(yaw)
        best = float("inf")
        for box in self.nav_scan_aabbs:
            tmin = -float("inf")
            tmax = float("inf")
            if abs(dx) < 1e-9:
                if ox < box["min_x"] or ox > box["max_x"]:
                    continue
            else:
                tx1 = (box["min_x"] - ox) / dx
                tx2 = (box["max_x"] - ox) / dx
                tmin = max(tmin, min(tx1, tx2))
                tmax = min(tmax, max(tx1, tx2))
            if abs(dy) < 1e-9:
                if oy < box["min_y"] or oy > box["max_y"]:
                    continue
            else:
                ty1 = (box["min_y"] - oy) / dy
                ty2 = (box["max_y"] - oy) / dy
                tmin = max(tmin, min(ty1, ty2))
                tmax = min(tmax, max(ty1, ty2))
            if tmax < max(tmin, 0.0):
                continue
            distance = tmin if tmin >= 0.0 else tmax
            if SCAN_RANGE_MIN <= distance <= SCAN_RANGE_MAX:
                best = min(best, distance)
        return best

    def publish_scan(self, stamp=None):
        if stamp is None:
            stamp = self.get_clock().now().to_msg()
        scan = LaserScan()
        scan.header.stamp = stamp
        scan.header.frame_id = "mid360_scan"
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = (scan.angle_max - scan.angle_min) / (SCAN_SAMPLES - 1)
        scan.time_increment = 0.0
        scan.scan_time = float(self.model.opt.timestep * SCAN_PERIOD_STEPS)
        scan.range_min = SCAN_RANGE_MIN
        scan.range_max = SCAN_RANGE_MAX
        ranges = []
        intensities = []
        origin = self.scan_origin()
        # Project a small vertical fan from the 3D MuJoCo scene into a 2D
        # LaserScan.  This approximates the obstacle projection a real Mid-360
        # pipeline would feed to Nav2: high-mounted rays can still mark a table
        # or box below the horizontal lidar plane.
        for index in range(SCAN_SAMPLES):
            local_angle = scan.angle_min + index * scan.angle_increment
            world_angle = self.base_yaw + local_angle
            distance = min(
                self.ray_scene_projection(origin, world_angle),
                self.ray_aabb_projection(origin, world_angle),
            )
            ranges.append(float(distance))
            intensities.append(1.0 if math.isfinite(distance) else 0.0)
        scan.ranges = ranges
        scan.intensities = intensities
        self.scan_pub.publish(scan)

    def publish_mid360_points(self, stamp=None):
        if stamp is None:
            stamp = self.get_clock().now().to_msg()
        origin = self.scan_origin()
        rotation = self.scan_rotation()
        points = []
        for pitch in MID360_VERTICAL_ANGLES:
            cos_pitch = math.cos(float(pitch))
            sin_pitch = math.sin(float(pitch))
            for index in range(SCAN_SAMPLES):
                local_angle = -math.pi + index * (2.0 * math.pi / (SCAN_SAMPLES - 1))
                ray_local = np.array([
                    cos_pitch * math.cos(local_angle),
                    cos_pitch * math.sin(local_angle),
                    sin_pitch,
                ], dtype=np.float64)
                ray_world = rotation @ ray_local
                distance = self.ray_scene_distance(origin, ray_world)
                if not (SCAN_RANGE_MIN <= distance <= SCAN_RANGE_MAX):
                    continue
                # mj_ray returns a distance along a ray expressed in the
                # MuJoCo world frame.  Publish the actual world-space hit in
                # odom rather than reconstructing it through the independently
                # maintained URDF sensor TF in RViz.  The MuJoCo world and our
                # odom frame share the same origin and axes.
                hit_world = origin + ray_world * distance
                points.append((float(hit_world[0]), float(hit_world[1]), float(hit_world[2])))
        cloud_header = Header()
        cloud_header.stamp = stamp
        cloud_header.frame_id = "odom"
        self.mid360_points_pub.publish(point_cloud2.create_cloud_xyz32(cloud_header, points))

    def publish_state(self, stamp=None):
        if stamp is None:
            stamp = self.get_clock().now().to_msg()
        msg = JointState()
        msg.header.stamp = stamp
        msg.name = self.names
        msg.position = [self.qpos(name) for name in self.names]
        msg.velocity = [self.qvel(name) for name in self.names]
        self.publisher.publish(msg)
        target_msg = JointState()
        target_msg.header.stamp = stamp
        target_msg.name = self.names
        target_msg.position = [float(self.target.get(name, self.qpos(name)))
                               for name in self.names]
        target_msg.velocity = [self.qvel(name) for name in self.names]
        target_msg.effort = [float(self.data.ctrl[self.actuator_ids[name]])
                             for name in self.names]
        self.target_pub.publish(target_msg)
        contacts = 0
        for index in range(self.data.ncon):
            contact = self.data.contact[index]
            bodies = [mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY,
                                        self.model.geom_bodyid[geom]) or ""
                      for geom in (contact.geom1, contact.geom2)]
            if "pickup_box" in bodies and any("_hand_" in body for body in bodies):
                contacts += 1
        self.contacts_pub.publish(Int32(data=contacts))
        # At least two distinct contact points is a measurable contact grasp
        # signal. It does not alter physics or attach the object.
        self.grasp_pub.publish(Bool(data=contacts >= 2))
        if self.tabletop_pick and self.box_body_id >= 0:
            truth = PoseStamped()
            truth.header.stamp, truth.header.frame_id = stamp, "odom"
            position = self.data.xpos[self.box_body_id]
            quaternion = self.data.xquat[self.box_body_id]  # MuJoCo w,x,y,z
            truth.pose.position.x, truth.pose.position.y, truth.pose.position.z = map(float, position)
            truth.pose.orientation.w = float(quaternion[0])
            truth.pose.orientation.x = float(quaternion[1])
            truth.pose.orientation.y = float(quaternion[2])
            truth.pose.orientation.z = float(quaternion[3])
            self.box_truth_pub.publish(truth)
        if self.base_qposadr is not None:
            base = PoseStamped()
            base.header.stamp, base.header.frame_id = stamp, "odom"
            base.pose.position.x = float(self.data.qpos[self.base_qposadr + 0])
            base.pose.position.y = float(self.data.qpos[self.base_qposadr + 1])
            base.pose.position.z = float(self.data.qpos[self.base_qposadr + 2])
            base.pose.orientation.w = float(self.data.qpos[self.base_qposadr + 3])
            base.pose.orientation.x = float(self.data.qpos[self.base_qposadr + 4])
            base.pose.orientation.y = float(self.data.qpos[self.base_qposadr + 5])
            base.pose.orientation.z = float(self.data.qpos[self.base_qposadr + 6])
            self.base_truth_pub.publish(base)

    def camera_info(self, stamp):
        info = CameraInfo()
        info.header.stamp = stamp
        info.header.frame_id = "d435_color_optical_frame"
        info.width, info.height = 640, 480
        fovy = np.deg2rad(69.0)
        fy = info.height / (2.0 * np.tan(fovy * 0.5))
        fx = fy
        cx, cy = info.width * 0.5, info.height * 0.5
        info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        return info, fx, fy, cx, cy

    def publish_camera(self):
        self.renderer.update_scene(self.data, camera="d435i")
        rgb = self.renderer.render()
        self.renderer.enable_depth_rendering()
        depth = self.renderer.render().astype(np.float32)
        self.renderer.disable_depth_rendering()
        stamp = self.get_clock().now().to_msg()
        info, fx, fy, cx, cy = self.camera_info(stamp)
        self.color_info_pub.publish(info)
        self.depth_info_pub.publish(info)
        color = Image()
        color.header = info.header
        color.height, color.width = rgb.shape[:2]
        color.encoding, color.is_bigendian = "rgb8", False
        color.step = color.width * 3
        color.data = rgb.tobytes()
        self.color_pub.publish(color)
        depth_msg = Image()
        depth_msg.header = info.header
        depth_msg.height, depth_msg.width = depth.shape
        depth_msg.encoding, depth_msg.is_bigendian = "32FC1", False
        depth_msg.step = depth_msg.width * 4
        depth_msg.data = depth.tobytes()
        self.depth_pub.publish(depth_msg)
        # Decimate to keep the ROS point cloud lightweight while preserving
        # the optical frame convention used by RealSense-compatible tools.
        v, u = np.mgrid[0:depth.shape[0]:4, 0:depth.shape[1]:4]
        distance = depth[::4, ::4]
        valid = np.isfinite(distance) & (distance > 0.05) & (distance < 5.0)
        # MuJoCo returns a ray distance, whereas ROS pinhole projection uses
        # optical Z.  Convert before producing the PointCloud2 message.
        xn = (u[valid] - cx) / fx
        yn = (v[valid] - cy) / fy
        z = distance[valid] / np.sqrt(1.0 + xn * xn + yn * yn)
        x = xn * z
        y = yn * z
        points = np.column_stack((x, y, z)).astype(np.float32)
        # A zero stamp asks TF consumers such as RViz for the latest available
        # torso transform.  Rendering takes longer than a physics step, so a
        # wall-clock stamp here can otherwise be a few milliseconds newer than
        # the latest /joint_states-derived transform and flicker to Error.
        cloud_header = Header()
        cloud_header.frame_id = info.header.frame_id
        self.points_pub.publish(point_cloud2.create_cloud_xyz32(cloud_header, points))

    def destroy_node(self):
        if self.viewer is not None:
            self.viewer.close()
        if self.renderer is not None:
            self.renderer.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = G1Mujoco()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
