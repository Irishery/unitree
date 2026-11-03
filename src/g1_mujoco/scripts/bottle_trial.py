#!/usr/bin/env python3
"""Record one stationary bottle-pick trial and save stage screenshots.

Run inside the MuJoCo container after the pick launch is up:

    python3 /ws/src/g1_mujoco/scripts/bottle_trial.py \
        --output /ws/trials/trial_01_center

The harness waits for a fresh RGB-D bottle pose, publishes /g1/pick/start,
records the ground-truth trajectory and writes a third-person PNG for each
selected sequence stage.
"""
import argparse
import csv
import json
import math
import time
from pathlib import Path

import cv2
import mujoco
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Int32, String


TABLE_TOP_Z = 0.755
BOTTLE_CENTRE_Z = 0.8725
CAPTURE_STAGES = (
    "ready", "approach_4", "hover", "align", "close",
    "lift_2", "lift_4", "hold", "lower_2", "release", "done",
)
STARTUP_TIMEOUT = 35.0
RUN_TIMEOUT = 90.0


def quaternion_matrix(x, y, z, w):
    q = np.asarray([w, x, y, z], dtype=np.float64)
    q /= max(np.linalg.norm(q), 1e-12)
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)],
        [2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w)],
        [2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)],
    ])


class BottleTrial(Node):
    def __init__(self, output, scene):
        super().__init__("g1_bottle_trial")
        self.output = Path(output)
        self.output.mkdir(parents=True, exist_ok=True)
        self.frames_dir = self.output / "frames"
        self.frames_dir.mkdir(parents=True, exist_ok=True)
        self.model = mujoco.MjModel.from_xml_path(scene)
        self.data = mujoco.MjData(self.model)
        self.renderer = mujoco.Renderer(self.model, width=640, height=480)
        self.camera = mujoco.MjvCamera()
        self.camera.lookat = [0.40, 0.0, 0.90]
        self.camera.distance = 1.9
        self.camera.elevation = -22
        self.camera.azimuth = 135
        self.joint_qadr = {}
        for jid in range(self.model.njnt):
            if self.model.jnt_type[jid] not in (
                    mujoco.mjtJoint.mjJNT_HINGE, mujoco.mjtJoint.mjJNT_SLIDE):
                continue
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, jid)
            if name:
                self.joint_qadr[name] = int(self.model.jnt_qposadr[jid])
        self.base_qadr = int(self.model.jnt_qposadr[mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_JOINT, "floating_base_joint")])
        self.bottle_qadr = int(self.model.jnt_qposadr[mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_JOINT, "pickup_bottle_free")])
        self.qpos = np.array(self.data.qpos)
        self.qpos[self.base_qadr:self.base_qadr + 7] = [0.0, 0.0, 0.793, 1.0, 0.0, 0.0, 0.0]
        self.joints = {}
        self.bottle_gt = None
        self.bottle_tilt_deg = None
        self.base_gt = None
        self.detected = False
        self.detected_pose = None
        self.detection_error = None
        self.detection_error_at_start = None
        self.detect_since = None
        self.status = None
        self.status_log = []
        self.trajectory = []
        self.images = 0
        self.started = False
        self.started_at = None
        self.finished_at = None
        self.start_republish_until = 0.0
        self.start_wall = time.time()
        self.last_trajectory_time = 0.0
        self.start_pub = self.create_publisher(Bool, "/g1/pick/start", 10)
        self.create_subscription(JointState, "/g1/joint_states", self.on_joints, 20)
        self.create_subscription(
            PoseStamped, "/g1/mujoco/evaluation/bottle_ground_truth", self.on_bottle_gt, 20)
        self.create_subscription(
            PoseStamped, "/g1/mujoco/evaluation/base_ground_truth", self.on_base_gt, 20)
        self.create_subscription(
            PoseStamped, "/g1/perception/bottle_pose", self.on_detected_pose, 20)
        self.create_subscription(Bool, "/g1/perception/bottle_detected", self.on_detected, 20)
        self.create_subscription(String, "/g1/pick/status", self.on_status, 200)
        self.create_subscription(Int32, "/g1/mujoco/hand_bottle_contacts", self.on_contacts, 20)
        self.create_subscription(Bool, "/g1/mujoco/physical_grasp", self.on_grasp, 20)
        self.contacts = 0
        self.grasp = False
        self.create_timer(0.1, self.tick)
        self.get_logger().info(
            f"bottle trial recorder ready; output={self.output}; waiting for RGB-D pose")

    def on_joints(self, message):
        self.joints = dict(zip(message.name, message.position))

    def on_bottle_gt(self, message):
        self.bottle_gt = message
        q = message.pose.orientation
        norm = math.sqrt(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z)
        if norm > 1e-9:
            w, x, y, z = q.w/norm, q.x/norm, q.y/norm, q.z/norm
            upright_z = 1.0 - 2.0 * (x*x + y*y)
            self.bottle_tilt_deg = math.degrees(
                math.acos(max(-1.0, min(1.0, upright_z))))

    def on_base_gt(self, message):
        self.base_gt = message

    def on_detected(self, message):
        self.detected = bool(message.data)
        if self.detected:
            if self.detect_since is None:
                self.detect_since = time.time()
        else:
            self.detect_since = None

    def on_detected_pose(self, message):
        self.detected_pose = message
        if self.bottle_gt is None:
            return
        p = message.pose.position
        world = np.array([p.x, p.y, p.z])
        if message.header.frame_id == "pelvis":
            world[2] += 0.793
        gt = self.bottle_gt.pose.position
        self.detection_error = float(np.linalg.norm(world - np.array([gt.x, gt.y, gt.z])))

    def on_status(self, message):
        stage = message.data
        if stage == self.status:
            return
        self.status = stage
        self.status_log.append((time.time(), stage))
        self.get_logger().info(f"status -> {stage}")
        if stage in CAPTURE_STAGES:
            self.snapshot(stage)
        if stage == "done" or stage.startswith("failed:"):
            self.finished_at = time.time()

    def on_contacts(self, message):
        self.contacts = int(message.data)

    def on_grasp(self, message):
        self.grasp = bool(message.data)

    def _apply_telemetry(self):
        if self.joints:
            for name, value in self.joints.items():
                adr = self.joint_qadr.get(name)
                if adr is not None:
                    self.qpos[adr] = value
        if self.base_gt is not None:
            p, q = self.base_gt.pose.position, self.base_gt.pose.orientation
            self.qpos[self.base_qadr:self.base_qadr + 7] = [p.x, p.y, p.z, q.w, q.x, q.y, q.z]
        if self.bottle_gt is not None:
            p, q = self.bottle_gt.pose.position, self.bottle_gt.pose.orientation
            self.qpos[self.bottle_qadr:self.bottle_qadr + 3] = [p.x, p.y, p.z]
            self.qpos[self.bottle_qadr + 3:self.bottle_qadr + 7] = [q.w, q.x, q.y, q.z]

    def snapshot(self, label):
        if self.bottle_gt is None:
            return
        self._apply_telemetry()
        self.data.qpos[:] = self.qpos
        mujoco.mj_forward(self.model, self.data)
        self.renderer.update_scene(self.data, camera=self.camera)
        image = self.renderer.render()
        path = self.frames_dir / f"{self.images:02d}_{label}.png"
        cv2.imwrite(str(path), cv2.cvtColor(image, cv2.COLOR_RGB2BGR))
        self.images += 1
        self.get_logger().info(f"screenshot {path}")

    def tick(self):
        now = time.time()
        if not self.started:
            if (self.detected and self.detect_since is not None
                    and now - self.detect_since > 0.8 and self.bottle_gt is not None):
                self.started = True
                self.started_at = now
                self.detection_error_at_start = self.detection_error
                self.start_republish_until = now + 2.5
                self.get_logger().info("publishing /g1/pick/start")
            elif now - self.start_wall > STARTUP_TIMEOUT:
                self.finish("no_fresh_rgbd_pose")
            return
        if now < self.start_republish_until:
            self.start_pub.publish(Bool(data=True))
        if self.bottle_gt is not None and now - self.last_trajectory_time > 0.2:
            self.last_trajectory_time = now
            p = self.bottle_gt.pose.position
            self.trajectory.append({
                "t": round(now - self.started_at, 3),
                "stage": self.status or "",
                "bottle_x": round(p.x, 4),
                "bottle_y": round(p.y, 4),
                "bottle_z": round(p.z, 4),
                "contacts": self.contacts,
                "grasp": int(self.grasp),
            })
        if self.finished_at is not None and now - self.finished_at > 1.5:
            self.finish("completed")
            return
        if now - self.started_at > RUN_TIMEOUT:
            self.finish("run_timeout")

    def finish(self, reason):
        if self.bottle_gt is not None:
            self.snapshot("final")
        summary = self.summarise(reason)
        (self.output / "summary.json").write_text(
            json.dumps(summary, indent=2) + "\n", encoding="utf-8")
        with (self.output / "trajectory.csv").open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(
                handle, fieldnames=list(self.trajectory[0].keys()) if self.trajectory
                else ["t", "stage", "bottle_x", "bottle_y", "bottle_z", "contacts", "grasp"])
            writer.writeheader()
            writer.writerows(self.trajectory)
        (self.output / "stages.txt").write_text(
            " -> ".join(stage for _, stage in self.status_log) + "\n", encoding="utf-8")
        self.get_logger().info(f"RESULT {json.dumps(summary, sort_keys=True)}")
        try:
            self.renderer.close()
        except Exception:
            pass
        rclpy.shutdown()

    def summarise(self, reason):
        zs = [row["bottle_z"] for row in self.trajectory]
        start_z = zs[0] if zs else None
        max_z = max(zs) if zs else None
        hold = [row["bottle_z"] for row in self.trajectory if row["stage"] == "hold"]
        final_z = zs[-1] if zs else None
        failures = []
        detection_error = (self.detection_error_at_start
                           if self.detection_error_at_start is not None
                           else self.detection_error)
        if detection_error is None or detection_error > 0.03:
            failures.append("detection_error")
        if final_z is None or abs(final_z - BOTTLE_CENTRE_Z) > 0.04:
            failures.append("not_on_table")
        if self.bottle_tilt_deg is None or self.bottle_tilt_deg > 25.0:
            failures.append("bottle_tipped")
        return {
            "outcome": "success" if not failures else "failure",
            "reason": reason,
            "detection_error_m": detection_error,
            "start_z": start_z,
            "lift_max_z": max_z,
            "lift_rise_m": (max_z - start_z) if zs else None,
            "hold_start_z": hold[0] if hold else None,
            "hold_min_z": min(hold) if hold else None,
            "hold_end_z": hold[-1] if hold else None,
            "final_z": final_z,
            "final_tilt_deg": self.bottle_tilt_deg,
            "tipped": bool(self.bottle_tilt_deg is None
                           or self.bottle_tilt_deg > 25.0),
            "screenshots": self.images,
            "stages": [stage for _, stage in self.status_log],
            "failures": failures,
        }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True)
    parser.add_argument(
        "--scene", default="/opt/unitree_ros/robots/g1_description/g1_29dof_with_dex3_bottle.xml")
    args = parser.parse_args()
    rclpy.init()
    node = BottleTrial(args.output, args.scene)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
