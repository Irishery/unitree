"""ROS 2 adapter for a contact-only MuJoCo G1 DEX3-1 tabletop scene."""
from pathlib import Path
import math
import os
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, Twist
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
WALK_POLICY_PATH = "/ws/models/walk/g1_12dof_motion.pt"
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
        self.tabletop_pick = bool(self.get_parameter("tabletop_pick").value)
        self.walk = bool(self.get_parameter("walk").value)
        self.viewer_lite = bool(self.get_parameter("viewer_lite").value)
        if self.walk and self.tabletop_pick:
            raise RuntimeError("walk:=true and tabletop_pick:=true are mutually exclusive")
        scene_file = (
            "g1_29dof_with_dex3_tabletop.xml"
            if self.tabletop_pick
            else "g1_29dof_with_dex3_nav.xml"
        )
        description = Path(os.environ.get("G1_DESCRIPTION_DIR", "/opt/unitree_ros/robots/g1_description"))
        scene_path = str(description / scene_file)
        temporary_scene_paths = []
        if self.walk:
            scene_path = self.build_walk_scene(scene_path)
            temporary_scene_paths.append(scene_path)
        if self.viewer_lite:
            scene_path = self.build_lite_scene(scene_path)
            temporary_scene_paths.append(scene_path)
        self.model = mujoco.MjModel.from_xml_path(scene_path)
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
        self.odom_vx = 0.0
        self.odom_vy = 0.0
        self.odom_wz = 0.0
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
            self.build_nav_blockers(inflate=WALK_BLOCKER_MARGIN) if self.walk else [])
        self.declare_parameter("publish_camera", True)
        self.publisher = self.create_publisher(JointState, "/g1/joint_states", 10)
        self.contacts_pub = self.create_publisher(Int32, "/g1/mujoco/hand_box_contacts", 10)
        self.grasp_pub = self.create_publisher(Bool, "/g1/mujoco/physical_grasp", 10)
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
        self.create_subscription(Twist, "/cmd_vel", self.set_cmd_vel, 10)
        self.create_subscription(Twist, "/cmd_vel_smoothed", self.set_smoothed_cmd_vel, 10)
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
        self.last_fall_log_time = 0.0
        self.get_logger().info(
            "MuJoCo walk mode: Unitree 12-DoF locomotion policy drives the legs "
            f"(pd at 500 Hz, policy at {int(1.0 / (WALK_CONTROL_DECIMATION * self.model.opt.timestep))} Hz); "
            "/cmd_vel is the velocity command and odom is derived from the physics root")

    def control_walk(self):
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
            kp, kd = (3.0, 0.18) if finger else (150.0, 4.0)
            limit = 0.9 if finger else 60.0
            torque = kp * (self.target[name] - self.qpos(name)) - kd * self.qvel(name)
            self.data.ctrl[actuator] = max(-limit, min(limit, torque))

    def walk_cmd(self):
        age = (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9
        if age > CMD_TIMEOUT:
            vx = vy = wz = 0.0
        else:
            vx = max(-0.35, min(0.35, float(self.cmd_vel.linear.x)))
            vy = max(-0.25, min(0.25, float(self.cmd_vel.linear.y)))
            wz = max(-0.8, min(0.8, float(self.cmd_vel.angular.z)))
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
        if abs(vx) < 1e-3 and abs(vy) < 1e-3 and abs(wz) < 1e-3:
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
        phase = ((self.step_count * self.model.opt.timestep) % WALK_GAIT_PERIOD) / WALK_GAIT_PERIOD
        obs[45] = math.sin(2.0 * math.pi * phase)
        obs[46] = math.cos(2.0 * math.pi * phase)
        import torch
        with torch.no_grad():
            self.walk_action = self.walk_policy(
                torch.from_numpy(obs).unsqueeze(0)).detach().numpy().squeeze()
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

    def step(self):
        if self.walk:
            self.control_walk()
        else:
            self.integrate_base()
            self.apply_base_pose()
            for name, actuator in self.actuator_ids.items():
                finger = "_hand_" in name
                kp, kd = (3.0, 0.18) if finger else (35.0, 1.8)
                limit = 0.9 if finger else 18.0
                torque = kp * (self.target[name] - self.qpos(name)) - kd * self.qvel(name)
                self.data.ctrl[actuator] = max(-limit, min(limit, torque))
        mujoco.mj_step(self.model, self.data)
        if self.walk:
            self.update_walk_odom()
        else:
            self.apply_base_pose()
        mujoco.mj_forward(self.model, self.data)
        self.step_count += 1
        if self.walk and self.step_count % WALK_CONTROL_DECIMATION == 0:
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
        if self.renderer is not None and self.step_count % 17 == 0:
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
        qvel[self.base_qveladr + 0] = self.odom_vx
        qvel[self.base_qveladr + 1] = self.odom_vy
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
                hit_local = ray_local * distance
                points.append((float(hit_local[0]), float(hit_local[1]), float(hit_local[2])))
        cloud_header = Header()
        cloud_header.stamp = stamp
        cloud_header.frame_id = "mid360_scan"
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
