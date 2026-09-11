"""Simulator-independent bimanual pick sequence and smooth joint timing."""
from dataclasses import dataclass
import math

import numpy as np

from .grasp_planning import (
    ARM_JOINTS, FINGER_OPEN, GraspParams, blend_rotation, finger_targets,
    grasp_frames, rotation_y)


READY = {
    "left": np.array([0.623, 0.886, 0.470, 0.289, -0.415, -0.445, -0.379]),
    "right": np.array([0.623, -0.886, -0.470, 0.289, 0.415, -0.445, 0.379]),
}
MIRROR_ARM = np.array([1.0, -1.0, -1.0, 1.0, -1.0, 1.0, -1.0])


@dataclass
class Segment:
    name: str
    duration: float
    arms: dict
    hands: dict


def _hands(closed, params):
    return {side: (finger_targets(side, params.finger_close_scale, params)
                   if closed else FINGER_OPEN[side].copy()) for side in ("left", "right")}


def build_pick_plan(kinematics, centre, yaw, params=None):
    """Build DETECT-to-RETREAT waypoints using only an observed box pose."""
    params = params or GraspParams()
    frames = grasp_frames(np.asarray(centre, dtype=float), yaw, params)
    ingress, descend, slide, straighten = {}, {}, {}, {}
    squeeze, front_seat, lift, hold = {}, {}, {}, {}
    arm = {}
    for side, sign in (("left", 1.0), ("right", -1.0)):
        seed = READY[side]
        start = np.array([0.05, sign * 0.40, 0.95])
        ingress[side] = []
        for index, alpha in enumerate(np.linspace(0.2, 1.0, 5)):
            position = start * (1.0 - alpha) + frames[side]["pregrasp"] * alpha
            q_init = (MIRROR_ARM * ingress["left"][index]
                      if side == "right" else seed)
            seed = kinematics.solve_arm_ik(
                side, position, frames[side]["approach_rotation"], q_init=q_init)
            ingress[side].append(seed)
        arm.setdefault(side, {})["pregrasp"] = seed
        q_init = MIRROR_ARM * arm["left"]["hover"] if side == "right" else seed
        seed = kinematics.solve_arm_ik(
            side, frames[side]["hover"], frames[side]["approach_rotation"], q_init=q_init)
        arm[side]["hover"] = seed
        descend[side] = []
        for index, alpha in enumerate(np.linspace(0.25, 1.0, 4)):
            position = ((1.0 - alpha) * frames[side]["hover"]
                        + alpha * frames[side]["outside_grasp"])
            q_init = (MIRROR_ARM * descend["left"][index]
                      if side == "right" else seed)
            seed = kinematics.solve_arm_ik(
                side, position, frames[side]["approach_rotation"], q_init=q_init)
            descend[side].append(seed)
        slide[side] = []
        for index, alpha in enumerate(np.linspace(0.125, 1.0, 8)):
            position = ((1.0 - alpha) * frames[side]["outside_grasp"]
                        + alpha * frames[side]["grasp"])
            q_init = MIRROR_ARM * slide["left"][index] if side == "right" else seed
            seed = kinematics.solve_arm_ik(
                side, position, frames[side]["approach_rotation"], q_init=q_init)
            slide[side].append(seed)
        q_init = MIRROR_ARM * arm["left"]["grasp"] if side == "right" else seed
        final_q = kinematics.solve_arm_ik(
            side, frames[side]["grasp"], frames[side]["rotation"], q_init=q_init)
        straighten[side] = [
            (1.0 - alpha) * seed + alpha * final_q
            for alpha in np.linspace(0.25, 1.0, 4)
        ]
        arm[side]["grasp"] = straighten[side][-1]
        seed = final_q
        squeeze[side] = []
        for index, alpha in enumerate(np.linspace(0.25, 1.0, 4)):
            position = ((1.0 - alpha) * frames[side]["grasp"]
                        + alpha * frames[side]["clamp"])
            q_init = MIRROR_ARM * squeeze["left"][index] if side == "right" else seed
            seed = kinematics.solve_arm_ik(
                side, position, frames[side]["rotation"], q_init=q_init)
            squeeze[side].append(seed)
        arm[side]["clamp"] = squeeze[side][-1]
        front_seat[side] = []
        for index, alpha in enumerate(np.linspace(0.25, 1.0, 4)):
            position = ((1.0 - alpha) * frames[side]["clamp"]
                        + alpha * frames[side]["carry"])
            q_init = (MIRROR_ARM * front_seat["left"][index]
                      if side == "right" else seed)
            seed = kinematics.solve_arm_ik(
                side, position, frames[side]["rotation"], q_init=q_init)
            front_seat[side].append(seed)
        arm[side]["carry"] = front_seat[side][-1]
        lift[side] = []
        elbow = f"{side}_elbow_joint"
        lift_bounds = {elbow: (params.lift_elbow_min, kinematics.ranges[elbow][1])}
        for index, dz in enumerate(np.linspace(0.02, params.lift_height, 6)):
            lift_alpha = dz / params.lift_height
            lift_rotation = (rotation_y(math.radians(params.lift_pitch_deg) * lift_alpha)
                             @ frames[side]["rotation"])
            q_init = MIRROR_ARM * lift["left"][index] if side == "right" else seed
            seed = kinematics.solve_arm_ik(
                side,
                frames[side]["carry"]
                + np.array([params.lift_forward * lift_alpha, 0.0, dz]),
                lift_rotation, q_init=q_init, joint_bounds=lift_bounds)
            lift[side].append(seed)
        hold_rotation = (rotation_y(math.radians(params.hold_pitch_deg))
                         @ frames[side]["rotation"])
        q_init = MIRROR_ARM * hold["left"] if side == "right" else seed
        hold[side] = kinematics.solve_arm_ik(
            side, frames[side]["lift"], hold_rotation, q_init=q_init,
            joint_bounds=lift_bounds)
    opened, closed = _hands(False, params), _hands(True, params)
    segments = [Segment("ready", 2.5, READY, opened)]
    for index in range(5):
        segments.append(Segment(f"approach_{index + 1}", 0.8,
                                {s: ingress[s][index] for s in ingress}, opened))
    segments += [
        Segment("hover", 1.5, {s: arm[s]["hover"] for s in arm}, opened),
    ]
    for index in range(4):
        segments.append(Segment(f"descend_{index + 1}", 0.6,
                                {s: descend[s][index] for s in descend}, opened))
    for index in range(8):
        segments.append(Segment(f"side_clamp_{index + 1}", 0.6,
                                {s: slide[s][index] for s in slide}, opened))
    for index in range(4):
        segments.append(Segment(f"straighten_{index + 1}", 0.6,
                                {s: straighten[s][index] for s in straighten}, opened))
    segments.append(Segment("close", 2.0, {s: arm[s]["grasp"] for s in arm}, closed))
    for index in range(4):
        segments.append(Segment(f"whole_hand_clamp_{index + 1}", 0.7,
                                {s: squeeze[s][index] for s in squeeze}, closed))
    if params.front_seat > 1e-6:
        for index in range(4):
            segments.append(Segment(f"thumb_front_seat_{index + 1}", 0.7,
                                    {s: front_seat[s][index] for s in front_seat}, closed))
    for index in range(6):
        segments.append(Segment(f"lift_{index + 1}", 2.5,
                                {s: lift[s][index] for s in lift}, closed))
    segments += [
        Segment("tilt_to_thumb", 4.0, hold, closed),
        Segment("settle", 15.0, hold, closed),
        Segment("hold", 30.0, hold, closed),
        Segment("level_before_place", 4.0,
                {s: lift[s][-1] for s in lift}, closed),
    ]
    for index in reversed(range(6)):
        target = {s: (lift[s][index - 1] if index else arm[s]["carry"]) for s in lift}
        segments.append(Segment(f"place_{6 - index}", 0.9, target, closed))
    # Returning to the original grasp pose already seats the box on the table.
    # Driving the wrists lower would scrape the straight fingers along it.
    lower = {side: arm[side]["carry"] for side in ("left", "right")}
    segments += [
        Segment("seat", 2.0, lower, closed),
        Segment("release", 1.5, lower, opened),
        Segment("retreat", 1.5, {s: arm[s]["pregrasp"] for s in arm}, opened),
        Segment("ready_after", 2.5, READY, opened),
    ]
    return segments


class SmoothSequence:
    """Quintic setpoint stream with explicit velocity/acceleration bounds."""

    def __init__(self, segments, initial_arms, initial_hands,
                 max_velocity=0.75, max_acceleration=1.5):
        self.segments = list(segments)
        self.current_arms = {s: np.asarray(initial_arms[s], float) for s in initial_arms}
        self.current_hands = {s: np.asarray(initial_hands[s], float) for s in initial_hands}
        self.max_velocity, self.max_acceleration = max_velocity, max_acceleration
        self.index, self.started = 0, 0.0
        self.start_arms = self.current_arms
        self.start_hands = self.current_hands

    @property
    def done(self):
        return self.index >= len(self.segments)

    @property
    def stage(self):
        return "done" if self.done else self.segments[self.index].name

    def _duration(self, segment):
        delta = max(np.max(np.abs(segment.arms[s] - self.start_arms[s])) for s in segment.arms)
        # maxima of the normalized quintic's first and second derivatives
        return max(segment.duration, 1.875 * delta / self.max_velocity,
                   math.sqrt(5.774 * delta / self.max_acceleration))

    def sample(self, now):
        if self.done:
            return self.current_arms, self.current_hands
        segment = self.segments[self.index]
        duration = self._duration(segment)
        u = min(1.0, max(0.0, (now - self.started) / duration))
        blend = 10*u**3 - 15*u**4 + 6*u**5
        arms = {s: self.start_arms[s] + blend * (segment.arms[s] - self.start_arms[s])
                for s in segment.arms}
        hands = {s: self.start_hands[s] + blend * (segment.hands[s] - self.start_hands[s])
                 for s in segment.hands}
        if u >= 1.0:
            self.current_arms, self.current_hands = arms, hands
            self.index += 1
            self.started = now
            self.start_arms = {s: v.copy() for s, v in arms.items()}
            self.start_hands = {s: v.copy() for s, v in hands.items()}
        return arms, hands
