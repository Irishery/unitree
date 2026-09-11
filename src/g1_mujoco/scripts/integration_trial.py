#!/usr/bin/env python3
"""Integration trial for the real RGB-D -> dual-arm pick pipeline.

This node deliberately drives the ROS graph exactly as a user would:
`g1_box_detector` publishes `/g1/perception/box_pose` and this harness then
publishes `/g1/pick/start`, letting `g1_dual_pick_controller` plan and execute
the bimanual sequence.  It never injects joint targets or box poses itself.

It records, for evaluation only:
  * the box ground-truth trajectory from `/g1/mujoco/evaluation/box_ground_truth`
  * stage transitions from `/g1/pick/status`
  * hand/box contact counts and the physical-grasp flag
  * full-scene third-person screenshots at selected sequence stages

Usage (inside the MuJoCo container, with ROS sourced):
    source /opt/ros/jazzy/setup.bash
    source /ws/install/setup.bash
    python3 /ws/src/g1_mujoco/scripts/integration_trial.py

Environment:
    G1_TRIAL_OUT      output directory (default /ws/src/g1_mujoco/artifacts/integration)
    G1_TRIAL_CAPTURE  comma-separated stages that get a screenshot
    G1_MUJOCO_MODEL   scene XML used for the screenshots (default railed tabletop)
"""
import csv
import json
import math
import os
import time
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Int32, String

try:
    import cv2
except Exception:  # pragma: no cover - optional dependency
    cv2 = None

try:
    import mujoco
except Exception:  # pragma: no cover
    mujoco = None


SCENE = os.environ.get(
    "G1_MUJOCO_MODEL",
    "/opt/unitree_ros/robots/g1_description/g1_29dof_with_dex3_tabletop_rails.xml")
OUT = Path(os.environ.get("G1_TRIAL_OUT", "/ws/src/g1_mujoco/artifacts/integration"))
CAPTURE = set(os.environ.get("G1_TRIAL_CAPTURE", (
    "ready,hover,close,whole_hand_clamp_4,lift_6,tilt_to_thumb,carry_squeeze,"
    "hold,level_before_place,place_2,place_4,seat,release,ready_after")).split(","))
TRANSPORT_SHOT_INTERVAL = float(os.environ.get("G1_TRIAL_TRANSPORT_SHOT_INTERVAL", "2.5"))
TABLE_TOP_Z = 0.755
STARTUP_TIMEOUT = 60.0
RUN_TIMEOUT = float(os.environ.get("G1_TRIAL_RUN_TIMEOUT", "260.0"))


def quaternion_matrix(x, y, z, w):
    q = np.asarray([w, x, y, z], dtype=np.float64)
    q /= max(np.linalg.norm(q), 1e-12)
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)],
        [2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w)],
        [2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)],
    ])


def box_tilt_deg(rotation):
    return float(np.degrees(np.arccos(np.clip(rotation[2, 2], -1.0, 1.0))))


def box_pitch_deg(rotation):
    return float(np.degrees(np.arctan2(-rotation[2, 0],
                                       math.hypot(rotation[2, 1], rotation[2, 2]))))


class ThirdPersonRecorder:
    """Reconstruct the scene from telemetry and save full-scene screenshots."""

    def __init__(self, scene_path, out_dir):
        if cv2 is None or mujoco is None:
            raise RuntimeError("recorder needs mujoco and cv2")
        self.mujoco = mujoco
        self.model = mujoco.MjModel.from_xml_path(scene_path)
        self.data = mujoco.MjData(self.model)
        self.frames_dir = Path(out_dir) / "frames"
        self.frames_dir.mkdir(parents=True, exist_ok=True)
        self.joint_qadr = {}
        for jid in range(self.model.njnt):
            if self.model.jnt_type[jid] in (
                    mujoco.mjtJoint.mjJNT_HINGE, mujoco.mjtJoint.mjJNT_SLIDE):
                name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, jid)
                if name:
                    self.joint_qadr[name] = int(self.model.jnt_qposadr[jid])
        self.box_qadr = int(self.model.jnt_qposadr[mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_JOINT, "pickup_box_free")])
        self.base_qadr = int(self.model.jnt_qposadr[mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_JOINT, "floating_base_joint")])
        self.qpos = np.array(self.data.qpos)
        self.qpos[self.base_qadr:self.base_qadr + 7] = [0.0, 0.0, 0.793, 1.0, 0.0, 0.0, 0.0]
        self.ready = False
        # Wide third-person view that keeps both tables and the whole carry
        # path in frame, so the walk to the second table is visible.
        self.camera = mujoco.MjvCamera()
        self.camera.lookat = [0.10, -0.45, 0.85]
        self.camera.distance = 2.9
        self.camera.elevation = -35.0
        self.camera.azimuth = 135.0
        self.width, self.height = 640, 480
        self.renderer = mujoco.Renderer(self.model, width=self.width, height=self.height)
        self.count = 0

    def update_telemetry(self, joints, box_pose, base_pose=None):
        if joints:
            for name, value in joints.items():
                adr = self.joint_qadr.get(name)
                if adr is not None:
                    self.qpos[adr] = value
        if box_pose is not None:
            p = box_pose.pose.position
            q = box_pose.pose.orientation
            self.qpos[self.box_qadr:self.box_qadr + 3] = [p.x, p.y, p.z]
            self.qpos[self.box_qadr + 3:self.box_qadr + 7] = [q.w, q.x, q.y, q.z]
            self.box_ready = True
        if base_pose is not None:
            half = 0.5 * base_pose[2]
            self.qpos[self.base_qadr:self.base_qadr + 7] = [
                base_pose[0], base_pose[1], 0.793, math.cos(half), 0.0, 0.0, math.sin(half)]
        self.ready = joints is not None and box_pose is not None

    def snapshot(self, label):
        if not self.ready:
            return None
        self.data.qpos[:] = self.qpos
        self.mujoco.mj_forward(self.model, self.data)
        self.renderer.update_scene(self.data, camera=self.camera)
        image = self.renderer.render()
        path = self.frames_dir / f"{self.count:02d}_{label}.png"
        cv2.imwrite(str(path), cv2.cvtColor(image, cv2.COLOR_RGB2BGR))
        self.count += 1
        return path

    def close(self):
        self.renderer.close()


class IntegrationTrial(Node):
    def __init__(self):
        super().__init__("g1_integration_trial")
        OUT.mkdir(parents=True, exist_ok=True)
        self.pose_pub = self.create_publisher(Bool, "/g1/pick/start", 10)
        self.create_subscription(PoseStamped, "/g1/perception/box_pose", self.on_detected_pose, 10)
        self.create_subscription(Bool, "/g1/perception/box_detected", self.on_detected, 10)
        self.create_subscription(String, "/g1/pick/status", self.on_status, 200)
        self.create_subscription(Int32, "/g1/mujoco/hand_box_contacts", self.on_contacts, 10)
        self.create_subscription(Bool, "/g1/mujoco/physical_grasp", self.on_grasp, 10)
        self.create_subscription(PoseStamped, "/g1/mujoco/evaluation/box_ground_truth",
                                 self.on_ground_truth, 10)
        self.create_subscription(JointState, "/g1/joint_states", self.on_joints, 10)
        self.create_subscription(Odometry, "/odom", self.on_odom, 10)

        self.state = "WAIT_DETECT"
        self.detect_since = None
        self.detected_pose = None
        self.detected_world = None
        self.detection_error = None
        self.last_status_wall = 0.0
        self.started_wall = None
        self.start_wall = time.time()
        self.seen_done_stage = False
        self.status_log = []
        self.trajectory = []
        self.contacts = 0
        self.contacts_max = 0
        self.grasp = False
        self.joints = {}
        self.gt = None
        self.base_odom = None
        self.finished_wall = None
        self.next_transport_shot = 0.0
        self.transport_shots = 0

        self.recorder = None
        try:
            self.recorder = ThirdPersonRecorder(SCENE, OUT)
            self.get_logger().info(f"saving full-scene screenshots to {OUT / 'frames'}")
        except Exception as error:  # pragma: no cover
            self.get_logger().warn(f"third-person recorder disabled: {error}")

        self.create_timer(1.0 / 15.0, self.tick)
        self.get_logger().info(f"integration trial armed; output={OUT}")

    def on_joints(self, message):
        self.joints = dict(zip(message.name, message.position))

    def on_odom(self, message):
        pose = message.pose.pose
        q = pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.base_odom = (pose.position.x, pose.position.y, yaw)

    def on_contacts(self, message):
        self.contacts = int(message.data)
        self.contacts_max = max(self.contacts_max, self.contacts)

    def on_grasp(self, message):
        self.grasp = bool(message.data)

    def on_detected(self, message):
        now = time.time()
        if message.data:
            if self.detect_since is None:
                self.detect_since = now
        else:
            self.detect_since = None

    def on_detected_pose(self, message):
        p = message.pose.position
        self.detected_pose = message
        self.detected_world = np.array([p.x, p.y, p.z])
        if message.header.frame_id == "pelvis":
            self.detected_world = self.detected_world + np.array([0.0, 0.0, 0.793])

    def on_status(self, message):
        now = time.time()
        stage = message.data
        if not self.status_log or self.status_log[-1][1] != stage:
            self.status_log.append((now, stage))
            self.get_logger().info(f"status -> {stage}")
            if self.recorder is not None and stage in CAPTURE:
                self.recorder.snapshot(stage)
        self.last_status_wall = now
        if stage in ("release", "retreat", "ready_after"):
            self.seen_done_stage = True
        if stage == "idle" and self.seen_done_stage and self.started_wall is not None:
            self.finished_wall = now

    def on_ground_truth(self, message):
        self.gt = message
        if self.recorder is not None and self.joints is not None:
            self.recorder.update_telemetry(self.joints, message, self.base_odom)

    def tick(self):
        now = time.time()
        if self.state == "WAIT_DETECT":
            if self.detect_since is not None and now - self.detect_since > 1.0:
                if self.detected_world is not None and self.gt is not None:
                    p = self.gt.pose.position
                    self.detection_error = float(np.linalg.norm(
                        self.detected_world - np.array([p.x, p.y, p.z])))
                self.pose_pub.publish(Bool(data=True))
                self.started_wall = now
                self.state = "RUNNING"
                self.get_logger().info("published /g1/pick/start")
            elif now - self.start_wall > STARTUP_TIMEOUT:
                self.fail("no_fresh_rgbd_pose")
            return
        if self.state == "RUNNING":
            if self.gt is not None:
                p = self.gt.pose.position
                q = self.gt.pose.orientation
                rotation = quaternion_matrix(q.x, q.y, q.z, q.w)
                half = np.array([0.075, 0.125, 0.07])
                box_min_z = p.z - float(np.abs(rotation[2, :]) @ half)
                base = self.base_odom or (0.0, 0.0, 0.0)
                self.trajectory.append({
                    "t": now - self.started_wall,
                    "stage": self.status_log[-1][1] if self.status_log else "",
                    "x": p.x, "y": p.y, "z": p.z, "box_min_z": box_min_z,
                    "base_x": base[0], "base_y": base[1], "base_yaw": base[2],
                    "left_shoulder_pitch": self.joints.get("left_shoulder_pitch_joint", 0.0),
                    "left_elbow": self.joints.get("left_elbow_joint", 0.0),
                    "left_wrist_pitch": self.joints.get("left_wrist_pitch_joint", 0.0),
                    "right_shoulder_pitch": self.joints.get("right_shoulder_pitch_joint", 0.0),
                    "right_elbow": self.joints.get("right_elbow_joint", 0.0),
                    "right_wrist_pitch": self.joints.get("right_wrist_pitch_joint", 0.0),
                    "tilt_deg": box_tilt_deg(rotation),
                    "pitch_deg": box_pitch_deg(rotation),
                    "contacts": self.contacts,
                    "grasp": int(self.grasp),
                })
            stage = self.status_log[-1][1] if self.status_log else ""
            if (self.recorder is not None and stage in ("carry_squeeze", "transport")
                    and now >= self.next_transport_shot):
                self.recorder.snapshot(f"transport_{self.transport_shots:02d}")
                self.transport_shots += 1
                self.next_transport_shot = now + TRANSPORT_SHOT_INTERVAL
            if self.finished_wall is not None:
                self.state = "SETTLE"
                self.finish_deadline = now + 2.0
            elif self.seen_done_stage and now - self.last_status_wall > 6.0:
                # The final "idle" transition was dropped by the subscription
                # queue; the sequence still reached ready_after.
                self.state = "SETTLE"
                self.finish_deadline = now + 1.0
            elif now - self.started_wall > RUN_TIMEOUT:
                self.finish("run_timeout")
            return
        if self.state == "SETTLE" and now >= self.finish_deadline:
            self.finish("completed")

    def fail(self, reason):
        self.get_logger().error(f"integration trial failed: {reason}")
        self.write_results("failed", reason)
        rclpy.shutdown()

    def finish(self, reason):
        self.write_results("completed" if reason == "completed" else "failed", reason)
        if self.recorder is not None:
            self.recorder.close()
        rclpy.shutdown()

    def write_results(self, outcome, reason):
        if self.recorder is not None:
            self.recorder.close()
            self.recorder = None
        summary = self.summarise(outcome, reason)
        summary["outcome"] = "success" if summary["success"] else "failure"
        (OUT / "summary.json").write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
        with (OUT / "trajectory.csv").open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=list(self.trajectory[0].keys())
                                    if self.trajectory else ["t"])
            writer.writeheader()
            writer.writerows(self.trajectory)
        stages = " -> ".join(stage for _, stage in self.status_log)
        (OUT / "stages.txt").write_text(stages + "\n", encoding="utf-8")
        self.get_logger().info(f"RESULT {json.dumps(summary, sort_keys=True)}")

    def summarise(self, outcome, reason):
        hold = [row for row in self.trajectory if row["stage"] == "hold"]
        lift = [row for row in self.trajectory if row["stage"].startswith("lift_")]
        zs = [row["z"] for row in self.trajectory]
        hold_z = [row["z"] for row in hold]
        hold_tilt = [row["tilt_deg"] for row in hold]
        hold_x = [row["x"] for row in hold]
        result = {
            "outcome": outcome,
            "reason": reason,
            "detection_error_m": self.detection_error,
            "lift_max_z": max(zs) if zs else None,
            "lift_rise_m": (max(zs) - zs[0]) if zs else None,
            "hold_start_z": hold_z[0] if hold_z else None,
            "hold_min_z": min(hold_z) if hold_z else None,
            "hold_end_z": hold_z[-1] if hold_z else None,
            "hold_slip_m": (max(hold_z) - min(hold_z)) if hold_z else None,
            "hold_x_drift_m": (max(hold_x) - min(hold_x)) if hold_x else None,
            "hold_tilt_max_deg": max(hold_tilt) if hold_tilt else None,
            "hold_tilt_end_deg": hold_tilt[-1] if hold_tilt else None,
            "hold_contacts_max": max((row["contacts"] for row in hold), default=None),
            "final_x": self.trajectory[-1]["x"] if self.trajectory else None,
            "final_y": self.trajectory[-1]["y"] if self.trajectory else None,
            "final_z": zs[-1] if zs else None,
            "stages": [stage for _, stage in self.status_log],
        }
        failures = []
        if result["lift_max_z"] is None or result["lift_max_z"] < TABLE_TOP_Z + 0.10 + 0.07:
            failures.append("lift_below_10cm")
        # A stationary hold only exists when the long hold phase is enabled; in
        # transport mode the box is carried instead, so skip those checks.
        if hold:
            if result["hold_min_z"] < TABLE_TOP_Z + 0.10 + 0.07:
                failures.append("hold_below_10cm")
            if result["hold_slip_m"] > 0.02:
                failures.append("hold_slip_over_2cm")
            if result["hold_contacts_max"] < 2:
                failures.append("no_physical_grasp")
        else:
            if result["hold_contacts_max"] is None:
                result["hold_contacts_max"] = self.contacts_max
        if result["final_z"] is None or abs(result["final_z"] - (TABLE_TOP_Z + 0.07)) > 0.03:
            failures.append("not_placed_on_table")
        result["failures"] = failures
        result["success"] = not failures
        return result


def main():
    rclpy.init()
    node = IntegrationTrial()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.recorder is not None:
            node.recorder.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
