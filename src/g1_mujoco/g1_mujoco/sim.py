"""ROS 2 adapter for a contact-only MuJoCo G1 DEX3-1 tabletop scene."""
from pathlib import Path
import math
import os

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
NAV_BASE_COLLISION_RADIUS = 0.26
NAV_BASE_COLLISION_MARGIN = 0.05
NAV_SCAN_AABB_MARGIN = 0.03


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
        self.tabletop_pick = bool(self.get_parameter("tabletop_pick").value)
        scene_file = (
            "g1_29dof_with_dex3_tabletop.xml"
            if self.tabletop_pick
            else "g1_29dof_with_dex3_nav.xml"
        )
        description = Path(os.environ.get("G1_DESCRIPTION_DIR", "/opt/unitree_ros/robots/g1_description"))
        self.model = mujoco.MjModel.from_xml_path(str(description / scene_file))
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
        for name, position in ARMS_AT_SIDES.items():
            self.data.qpos[self.model.jnt_qposadr[self.joint_ids[name]]] = position
        mujoco.mj_forward(self.model, self.data)
        self.target = {name: self.qpos(name) for name in self.names}
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
        self.declare_parameter("publish_camera", True)
        self.publisher = self.create_publisher(JointState, "/g1/joint_states", 10)
        self.contacts_pub = self.create_publisher(Int32, "/g1/mujoco/hand_box_contacts", 10)
        self.grasp_pub = self.create_publisher(Bool, "/g1/mujoco/physical_grasp", 10)
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.scan_pub = self.create_publisher(LaserScan, "/scan", 10)
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
            if self.get_parameter("viewer_lite").value:
                # Software-GL laptops render the textured G1 meshes slowly.
                # Shadows and textures are by far the heaviest part of the
                # viewer frame; dropping them keeps the physics view cheap.
                # NOTE: no local "import mujoco" here - it would shadow the
                # module-level import and crash __init__ with
                # UnboundLocalError at the MjModel load above.
                self.viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_TEXTURE] = False
                self.viewer.user_scn.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = False
                self.model.vis.quality.shadowsize = 0
        self.renderer = None
        if self.get_parameter("publish_camera").value:
            self.renderer = mujoco.Renderer(self.model, width=640, height=480)
        self.timer = self.create_timer(0.002, self.step)
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

    def build_nav_blockers(self):
        blockers = []
        inflate = NAV_BASE_COLLISION_RADIUS + NAV_BASE_COLLISION_MARGIN
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
        for blocker in self.nav_blockers:
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

    def step(self):
        self.integrate_base()
        self.apply_base_pose()
        for name, actuator in self.actuator_ids.items():
            finger = "_hand_" in name
            kp, kd = (3.0, 0.18) if finger else (35.0, 1.8)
            limit = 0.9 if finger else 18.0
            torque = kp * (self.target[name] - self.qpos(name)) - kd * self.qvel(name)
            self.data.ctrl[actuator] = max(-limit, min(limit, torque))
        mujoco.mj_step(self.model, self.data)
        self.apply_base_pose()
        mujoco.mj_forward(self.model, self.data)
        self.step_count += 1
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

    def ray_scene_projection(self, origin, yaw):
        best = float("inf")
        geomid = np.array([-1], dtype=np.int32)
        for pitch in MID360_VERTICAL_ANGLES:
            cos_pitch = math.cos(float(pitch))
            ray = np.array([
                cos_pitch * math.cos(yaw),
                cos_pitch * math.sin(yaw),
                math.sin(float(pitch)),
            ], dtype=np.float64)
            distance = mujoco.mj_ray(
                self.model, self.data, origin, ray, self.nav_scan_geomgroup, 1, -1, geomid)
            if distance < 0.0:
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
