# register_env.py

import gymnasium as gym
from Train.TR_one_ball_env import IsaacOneBallEnv

gym.register(
    id="Isaac-OneBall-v0",
    entry_point="Train.TR_one_ball_env:IsaacOneBallEnv",
    disable_env_checker=True,
)
