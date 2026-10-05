"""Gym environment: drive the real rig's tip to a randomly chosen stone from
the real 8-stone benchtop grid. First milestone toward computer-assisted
targeting -- state-based (tip/target positions), not vision-based. Getting
this working is the prerequisite for the harder version (drive off the
camera image instead of ground-truth positions) -- there's no point
debugging a vision pipeline and an RL policy at the same time.

    ~/isaacsim/python.sh training/reach_task.py        # smoke-test: random actions, no training

For actual training see train_reach.py (stable-baselines3 PPO).

WHAT'S VERIFIED vs. NOT, split deliberately:
  - reach_math.py (action scaling, joint-target integration, reward,
    observation) is plain Python, no Isaac Sim needed -- covered by
    test_reach_math.py, which runs and passes in this environment.
  - This file (the Gym env wrapping Isaac's World/physics stepping, and the
    real rig's joint-drive attributes) is UNVERIFIED like the rest of
    ros2/isaac -- no Isaac Sim install available here. The real-rig loading
    and joint-path code it calls into (_isaac_common.py) IS checked against
    plain USD; see assets/ASSET-NOTES.md for exactly what that covers.

REQUIRES the real rig assets (assets/glidar_robot/) -- there's no
placeholder-URDF version of this task, since the placeholder URDF has no
camera-less "tip" concept worth reaching toward and no real target
positions to reach for either.
"""

from __future__ import annotations

import random
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))  # for _isaac_common
sys.path.insert(0, str(Path(__file__).resolve().parent))  # for reach_math

import numpy as np

from reach_math import (
    MAX_EPISODE_STEPS,
    JointTargets,
    build_observation,
    compute_reward,
    integrate_targets,
    scale_action,
)

# One control step's worth of simulated time. Isaac's default physics step
# is commonly 1/60s or 1/240s depending on install config; 1/30s here is a
# deliberately coarse starting point (matches the real rig's own control
# loop cadence -- see python/config/system.yaml's motion.loop_hz: 50 and
# the IROS paper's communication frequency of 30fps) rather than something
# tuned against an actual running sim, which wasn't available to tune it
# against.
CONTROL_DT_S = 1.0 / 30.0
PHYSICS_STEPS_PER_CONTROL_STEP = 4


class ReachEnv:
    """A standard Gym-shaped env (reset/step/observation_space/action_space)
    without taking a hard gymnasium.Env dependency at import time -- keeps
    this importable (though not runnable) without gymnasium installed,
    matching reach_math.py's own zero-dependency stance. train_reach.py
    wraps this in a thin gymnasium.Env adapter instead of inheriting here.
    """

    observation_size = 6
    action_size = 3

    def __init__(self, world, stage, common, seed: int | None = None) -> None:
        self._world = world
        self._stage = stage
        self._common = common
        self._rng = random.Random(seed)
        self._targets = JointTargets()
        self._target_pos = (0.0, 0.0, 0.0)
        self._step_count = 0

    def reset(self) -> np.ndarray:
        self._world.reset()
        self._targets = JointTargets()
        self._apply_targets()

        stone_positions = self._common.stone_grid_positions(self._tip_position())
        self._target_pos = self._rng.choice(stone_positions)
        self._step_count = 0

        return self._observation()

    def step(self, action: np.ndarray):
        rates = scale_action(tuple(float(a) for a in action))
        self._targets = integrate_targets(self._targets, rates, CONTROL_DT_S)
        self._apply_targets()

        for _ in range(PHYSICS_STEPS_PER_CONTROL_STEP):
            self._world.step(render=False)

        self._step_count += 1
        tip_pos = self._tip_position()
        reward, success = compute_reward(tip_pos, self._target_pos)
        terminated = success
        truncated = self._step_count >= MAX_EPISODE_STEPS

        return self._observation(), reward, terminated, truncated, {"success": success}

    def _apply_targets(self) -> None:
        self._common.set_real_linear_target(self._stage, self._targets.linear_m)
        self._common.set_real_rotation_target(self._stage, self._targets.rotation_deg)
        self._common.set_real_flexion_target(self._stage, self._targets.flexion_deg)

    def _tip_position(self) -> tuple[float, float, float]:
        pos = self._common.get_real_tip_world_position(self._stage)
        return (pos[0], pos[1], pos[2])

    def _observation(self) -> np.ndarray:
        tip_pos = self._tip_position()
        obs = build_observation(self._targets, tip_pos, self._target_pos)
        return np.array(obs, dtype=np.float32)


def _smoke_test() -> None:
    """Random actions for a few episodes -- sanity-checks the env plumbing
    runs end to end, not that the task is learnable or that the physics
    behaves as expected (see the module docstring on what's unverified)."""
    from isaacsim import SimulationApp

    simulation_app = SimulationApp({"headless": False})

    try:
        from isaacsim.core.api import World

        import _isaac_common as common

        if not common.real_rig_available():
            print("[katzlab] real rig assets not found -- see assets/ASSET-NOTES.md")
            return

        world = World(stage_units_in_meters=1.0)
        world.scene.add_default_ground_plane()
        common.load_real_rig(world.stage)
        world.reset()

        env = ReachEnv(world, world.stage, common, seed=0)
        obs = env.reset()
        print("[katzlab] reset -> obs:", obs)

        for episode in range(3):
            obs = env.reset()
            total_reward = 0.0
            for _ in range(50):
                action = np.random.uniform(-1.0, 1.0, size=env.action_size)
                obs, reward, terminated, truncated, info = env.step(action)
                total_reward += reward
                if terminated or truncated:
                    break
            print(f"[katzlab] episode {episode}: total_reward={total_reward:.4f} info={info}")
    finally:
        simulation_app.close()


if __name__ == "__main__":
    _smoke_test()
