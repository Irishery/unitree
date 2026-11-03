"""Simulator-independent stationary single-arm bottle sequence."""
from dataclasses import dataclass
import math

import numpy as np

from .grasp_planning import (
    ARM_JOINTS, FINGER_OPEN, GraspParams, finger_targets, grasp_frames)


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


def _hands(closed, params, side):
    hands = {name: FINGER_OPEN[name].copy() for name in ("left", "right")}
    if closed:
        hands[side] = finger_targets(side, params.finger_close_scale, params)
    return hands


def _interp(start, end, alpha):
    return {side: (1.0 - alpha) * start[side] + alpha * end[side]
            for side in start}


def build_pick_plan(kinematics, centre, yaw, params=None, side="right"):
    """Build a stationary one-arm approach/close/lift/lower/release sequence.

    The selected hand performs the pick; the other arm stays in READY.  The
    bottle is centred between the robot and the table edge, and the base never
    moves.
    """
    if side not in ("left", "right"):
        raise ValueError(f"side must be 'left' or 'right', got {side!r}")
    params = params or GraspParams()
    other = "left" if side == "right" else "right"
    frames = grasp_frames(np.asarray(centre, dtype=float), yaw, params)
    seed = READY[side]
    arm = {}
    for name in ("pregrasp", "hover", "align", "grasp", "lift", "clear"):
        solved = kinematics.solve_arm_ik(
            side, frames[side][name], frames[side]["rotation"], q_init=seed)
        arm[name] = solved
        seed = solved

    def pose(values):
        return {side: values, other: READY[other]}

    opened, closed = _hands(False, params, side), _hands(True, params, side)
    segments = [Segment("ready", 2.5, READY, opened)]
    for index in range(1, 5):
        segments.append(Segment(
            f"approach_{index}", 0.8,
            _interp(READY, pose(arm["pregrasp"]), index / 4.0), opened))
    segments.append(Segment("hover", 1.0, pose(arm["hover"]), opened))
    for index in range(1, 4):
        segments.append(Segment(
            f"advance_{index}", 0.7,
            _interp(pose(arm["hover"]), pose(arm["align"]), index / 3.0), opened))
    segments.append(Segment("align", 1.0, pose(arm["align"]), opened))
    for index in range(1, 4):
        segments.append(Segment(
            f"seat_{index}", 0.7,
            _interp(pose(arm["align"]), pose(arm["grasp"]), index / 3.0), opened))
    segments.append(Segment("seat_settle", 1.5, pose(arm["grasp"]), opened))
    segments.append(Segment("close", 2.0, pose(arm["grasp"]), closed))
    for index in range(1, 5):
        segments.append(Segment(
            f"lift_{index}", 1.5,
            _interp(pose(arm["grasp"]), pose(arm["lift"]), index / 4.0), closed))
    segments.append(Segment("hold", 10.0, pose(arm["lift"]), closed))
    for index in range(1, 5):
        segments.append(Segment(
            f"lower_{index}", 1.5,
            _interp(pose(arm["lift"]), pose(arm["grasp"]), index / 4.0), closed))
    # Move the still-closed hand clear before opening it.  Opening the fingers
    # while they are touching the bottle is what tipped the narrow free body.
    segments.append(Segment("clear", 1.5, pose(arm["clear"]), closed))
    segments.append(Segment("release", 2.0, pose(arm["clear"]), opened))
    segments.append(Segment("retreat", 1.5, pose(arm["pregrasp"]), opened))
    segments.append(Segment("ready_after", 2.0, READY, opened))
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

    @property
    def current_segment(self):
        return None if self.done else self.segments[self.index]

    def advance(self, now):
        """Force-complete the current segment and move to the next one."""
        if self.done:
            return
        segment = self.segments[self.index]
        self.current_arms = {s: np.asarray(v, float).copy() for s, v in segment.arms.items()}
        self.current_hands = {s: np.asarray(v, float).copy() for s, v in segment.hands.items()}
        self.start_arms = {s: v.copy() for s, v in self.current_arms.items()}
        self.start_hands = {s: v.copy() for s, v in self.current_hands.items()}
        self.index += 1
        self.started = now

    def _duration(self, segment):
        delta = max(np.max(np.abs(segment.arms[s] - self.start_arms[s])) for s in segment.arms)
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
