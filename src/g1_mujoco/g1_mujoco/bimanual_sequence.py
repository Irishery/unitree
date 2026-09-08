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
    ingress, slide, lift = {}, {}, {}
    arm = {}
    for side, sign in (("left", 1.0), ("right", -1.0)):
        seed = READY[side]
        start = np.array([0.05, sign * 0.40, 0.95])
        ingress[side] = []
        for alpha in np.linspace(0.2, 1.0, 5):
            position = start * (1.0 - alpha) + frames[side]["pregrasp"] * alpha
            seed = kinematics.solve_arm_ik(
                side, position, frames[side]["approach_rotation"], q_init=seed)
            ingress[side].append(seed)
        arm.setdefault(side, {})["pregrasp"] = seed
        seed = kinematics.solve_arm_ik(
            side, frames[side]["hover"], frames[side]["approach_rotation"], q_init=seed)
        arm[side]["hover"] = seed
        slide[side] = []
        for alpha in np.linspace(0.125, 1.0, 8):
            position = ((1.0 - alpha) * frames[side]["hover"]
                        + alpha * frames[side]["grasp"])
            rotation = blend_rotation(
                frames[side]["approach_rotation"], frames[side]["rotation"], alpha)
            seed = kinematics.solve_arm_ik(side, position, rotation, q_init=seed)
            slide[side].append(seed)
        arm[side]["grasp"] = slide[side][-1]
        lift[side] = []
        for dz in np.linspace(0.04, params.lift_height, 6):
            lift_alpha = dz / params.lift_height
            lift_rotation = (rotation_y(math.radians(params.lift_pitch_deg) * lift_alpha)
                             @ frames[side]["rotation"])
            seed = kinematics.solve_arm_ik(
                side,
                frames[side]["grasp"]
                + np.array([params.lift_forward * lift_alpha, 0.0, dz]),
                lift_rotation, q_init=seed)
            lift[side].append(seed)
    opened, closed = _hands(False, params), _hands(True, params)
    segments = [Segment("ready", 2.5, READY, opened)]
    for index in range(5):
        segments.append(Segment(f"approach_{index + 1}", 0.8,
                                {s: ingress[s][index] for s in ingress}, opened))
    segments += [
        Segment("hover", 1.5, {s: arm[s]["hover"] for s in arm}, opened),
    ]
    for index in range(8):
        segments.append(Segment(f"side_clamp_{index + 1}", 0.6,
                                {s: slide[s][index] for s in slide}, opened))
    segments.append(Segment("close", 2.0, {s: arm[s]["grasp"] for s in arm}, closed))
    for index in range(6):
        segments.append(Segment(f"lift_{index + 1}", 2.5,
                                {s: lift[s][index] for s in lift}, closed))
    segments += [
        Segment("settle", 15.0, {s: lift[s][-1] for s in lift}, closed),
        Segment("hold", 30.0, {s: lift[s][-1] for s in lift}, closed),
    ]
    for index in reversed(range(6)):
        target = {s: (lift[s][index - 1] if index else arm[s]["grasp"]) for s in lift}
        segments.append(Segment(f"place_{6 - index}", 0.9, target, closed))
    # Returning to the original grasp pose already seats the box on the table.
    # Driving the wrists lower would scrape the straight fingers along it.
    lower = {side: arm[side]["grasp"] for side in ("left", "right")}
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
