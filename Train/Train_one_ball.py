# train_isaac_one_ball.py

import argparse
import gymnasium as gym 
from stable_baselines3 import PPO
from stable_baselines3.common.env_checker import check_env

# Import your environment registration
import register_env

def main():
    # Create the environment
    env = gym.make("Isaac-OneBall-v0")

    # Optional: check if the environment follows Gym's interface
    check_env(env)

    # Initialize the PPO model
    model = PPO("MlpPolicy", env, verbose=1)

    # Train the model
    model.learn(total_timesteps=100000)

    # Save the trained model
    model.save("ppo_isaac_one_ball")

    # Close the environment
    env.close()

if __name__ == "__main__":
    main()
