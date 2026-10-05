"""Train a PPO policy on the reach task: drive the real rig's tip to a
randomly chosen stone from the real 8-stone benchtop grid.

    ~/isaacsim/python.sh training/train_reach.py --total-timesteps 200000

Needs stable-baselines3 installed in Isaac Sim's own Python environment
(``~/isaacsim/python.sh -m pip install stable-baselines3``) -- picked
because it's the standard, well-documented pairing with a Gym-shaped env
like ReachEnv, not because anything about this task needs PPO specifically.
If Isaac Lab (the newer NVIDIA RL framework, née OmniIsaacGymEnvs) is
already part of your install, it would also work and would vectorize
better (many parallel envs in one sim) -- not used here to keep this one
script's dependency surface small and avoid committing to a fast-moving
API this session had no way to verify.

UNVERIFIED like the rest of ros2/isaac/training: never run against a real
Isaac Sim + stable-baselines3 install. What IS verified -- the reward/
observation/action math this trains against -- lives in reach_math.py and
is covered by test_reach_math.py.

Given how slow a from-scratch policy is likely to be on this reward alone,
reasonable expectations for a FIRST run: don't expect convergence to
reliable stone-reaching in one sitting. Watch mean_episode_reward trend up
and mean_episode_length trend down (fewer steps to terminate = reaching
the target rather than always timing out) as the first useful signal that
anything is working at all, before worrying about final performance.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))  # for _isaac_common
sys.path.insert(0, str(Path(__file__).resolve().parent))  # for reach_task, reach_math


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--total-timesteps", type=int, default=200_000)
    parser.add_argument("--save-path", type=str, default="./reach_ppo.zip")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--headless", action="store_true")
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    from isaacsim import SimulationApp

    simulation_app = SimulationApp({"headless": args.headless})

    try:
        import gymnasium as gym
        import numpy as np
        from isaacsim.core.api import World
        from stable_baselines3 import PPO
        from stable_baselines3.common.env_checker import check_env

        import _isaac_common as common
        from reach_task import ReachEnv

        if not common.real_rig_available():
            print("[katzlab] real rig assets not found -- see ../assets/ASSET-NOTES.md")
            return

        world = World(stage_units_in_meters=1.0)
        world.scene.add_default_ground_plane()
        common.load_real_rig(world.stage)
        world.reset()

        raw_env = ReachEnv(world, world.stage, common, seed=args.seed)
        env = GymnasiumAdapter(raw_env)

        # Sanity-checks the env against Gym's own API contract (shapes,
        # dtypes, reset/step signatures) before spending any training time
        # on it -- cheap, and the standard first thing to do with a new env.
        check_env(env, warn=True)

        model = PPO("MlpPolicy", env, verbose=1, seed=args.seed)
        model.learn(total_timesteps=args.total_timesteps)
        model.save(args.save_path)
        print(f"[katzlab] saved policy to {args.save_path}")
    finally:
        simulation_app.close()


class GymnasiumAdapter:
    """Wraps ReachEnv (deliberately gymnasium-free, see its own docstring)
    in gymnasium's Env interface, which stable-baselines3 actually needs."""

    metadata = {"render_modes": []}

    def __init__(self, inner) -> None:
        import gymnasium as gym
        import numpy as np

        self._inner = inner
        self.observation_space = gym.spaces.Box(
            low=-np.inf, high=np.inf, shape=(inner.observation_size,), dtype=np.float32
        )
        self.action_space = gym.spaces.Box(
            low=-1.0, high=1.0, shape=(inner.action_size,), dtype=np.float32
        )

    def reset(self, *, seed=None, options=None):
        obs = self._inner.reset()
        return obs, {}

    def step(self, action):
        return self._inner.step(action)

    def render(self):
        pass

    def close(self):
        pass


if __name__ == "__main__":
    main()
