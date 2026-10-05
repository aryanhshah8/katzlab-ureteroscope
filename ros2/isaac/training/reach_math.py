"""Pure math for the reach task -- action scaling, joint target integration,
reward, and observation construction. No Isaac Sim, no pxr, nothing that
can't run and be tested anywhere plain Python runs (see test_reach_math.py).

Kept separate from reach_task.py on purpose: this is the part of the reach
task that's actually possible to verify in this environment (no Isaac Sim
install here) -- reach_task.py itself (the Gym env wrapping Isaac's World/
physics stepping) is unverified like the rest of ros2/isaac, but the control
logic it calls into is not.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

# Real max rates, IROS 2026 paper Table I (see ../assets/ASSET-NOTES.md).
LINEAR_MAX_RATE_M_S = 0.002
ROTATION_MAX_RATE_DEG_S = 30.0
FLEXION_MAX_RATE_DEG_S = 35.0

# Real ranges -- CAD mechanical limits (linear, rotation) or paper-stated
# software limit (flexion); see ASSET-NOTES.md for the CAD-vs-paper
# discrepancy on rotation range.
LINEAR_RANGE_M = (-0.123, 0.123)
ROTATION_RANGE_DEG = (-153.5526885986328, 153.5526885986328)
FLEXION_RANGE_DEG = (-270.0, 270.0)

# Success = tip within this distance of the target stone's center.
# Placeholder -- no measured real positioning accuracy to base this on yet
# (the IROS 2026 paper says as much: Table I's tolerances are software
# control parameters, not measured physical accuracy). 5mm is a reasonable
# first target given the real stone scale (1-6mm radius, see
# _isaac_common.STONE_SCALE_RANGE_M) -- tune once there's real data.
SUCCESS_TOLERANCE_M = 0.005

# Episode length cap, in environment steps.
MAX_EPISODE_STEPS = 300


@dataclass
class JointTargets:
    linear_m: float = 0.0
    rotation_deg: float = 0.0
    flexion_deg: float = 0.0

    def as_tuple(self) -> tuple[float, float, float]:
        return (self.linear_m, self.rotation_deg, self.flexion_deg)


def _clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def scale_action(action: tuple[float, float, float]) -> tuple[float, float, float]:
    """Normalised [-1, 1] action -> (linear m/s, rotation deg/s, flexion
    deg/s), clamped to [-1, 1] first in case the policy over/undershoots."""
    linear_a, rotation_a, flexion_a = (_clamp(a, -1.0, 1.0) for a in action)
    return (
        linear_a * LINEAR_MAX_RATE_M_S,
        rotation_a * ROTATION_MAX_RATE_DEG_S,
        flexion_a * FLEXION_MAX_RATE_DEG_S,
    )


def integrate_targets(
    current: JointTargets, rates: tuple[float, float, float], dt_s: float
) -> JointTargets:
    """One control-step integration of commanded rates into new absolute
    joint targets, clamped to the real ranges -- mirrors how katzlab's own
    control.py integrates flexion rate into an absolute target
    (flexion_target_deg), just applied to all three axes here since this
    task commands velocity on all three rather than position-stepping two
    of them."""
    linear_rate, rotation_rate, flexion_rate = rates
    return JointTargets(
        linear_m=_clamp(
            current.linear_m + linear_rate * dt_s, LINEAR_RANGE_M[0], LINEAR_RANGE_M[1]
        ),
        rotation_deg=_clamp(
            current.rotation_deg + rotation_rate * dt_s,
            ROTATION_RANGE_DEG[0],
            ROTATION_RANGE_DEG[1],
        ),
        flexion_deg=_clamp(
            current.flexion_deg + flexion_rate * dt_s, FLEXION_RANGE_DEG[0], FLEXION_RANGE_DEG[1]
        ),
    )


def compute_reward(
    tip_pos: tuple[float, float, float],
    target_pos: tuple[float, float, float],
    success_tolerance_m: float = SUCCESS_TOLERANCE_M,
) -> tuple[float, bool]:
    """Dense negative-distance reward plus a one-off success bonus. Returns
    (reward, success). Dense rather than sparse on purpose: the real rig's
    linear axis alone moves at up to 2mm/s, so a sparse "reward only on
    exact success" signal over a 0.123m range would be a brutal exploration
    problem for a first milestone -- distance shaping is the standard fix
    and the obvious one to start with before trying anything fancier.
    """
    dx = tip_pos[0] - target_pos[0]
    dy = tip_pos[1] - target_pos[1]
    dz = tip_pos[2] - target_pos[2]
    distance = math.sqrt(dx * dx + dy * dy + dz * dz)

    success = distance <= success_tolerance_m
    reward = -distance + (10.0 if success else 0.0)
    return reward, success


def build_observation(
    targets: JointTargets, tip_pos: tuple[float, float, float], target_pos: tuple[float, float, float]
) -> tuple[float, ...]:
    """(linear_norm, rotation_norm, flexion_norm, dx, dy, dz) -- joint
    targets normalised to [-1, 1] against their real ranges (so the policy
    network sees comparable-scale inputs across axes with very different
    physical units/ranges), plus the raw tip-to-target offset in metres,
    which is small enough (order 0.01-0.1m) not to need its own
    normalisation for a first milestone.
    """

    def normalize(value: float, lo: float, hi: float) -> float:
        mid = (lo + hi) / 2.0
        half_range = (hi - lo) / 2.0
        return (value - mid) / half_range if half_range else 0.0

    return (
        normalize(targets.linear_m, *LINEAR_RANGE_M),
        normalize(targets.rotation_deg, *ROTATION_RANGE_DEG),
        normalize(targets.flexion_deg, *FLEXION_RANGE_DEG),
        tip_pos[0] - target_pos[0],
        tip_pos[1] - target_pos[1],
        tip_pos[2] - target_pos[2],
    )
