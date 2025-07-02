# verify_TR_oneball.py

from isaacsim import SimulationApp
simulation_app = SimulationApp({"renderer": "RaytracedLighting", "headless": False})

import os
import torch
from stable_baselines3 import PPO
import numpy as np
# import omni.graph.core as og
# print(og.get_node_type("isaacsim.ros2.bridge.ROS2SubscribeJointState"))
# print(og.get_node_type("isaacsim.ros2.bridge.ROS2PublishJointState"))
from Train.TR_one_ball_env import IsaacOneBallEnv

# === 1. Set paths ===
model_path = "logs_tensorboard_2025-06-19_15-55-20/best_model/best_model.zip"
# model_path = "./Hitted_wrong/best_model/best_model.zip"

# === 2. Create environment ===
env = IsaacOneBallEnv()

# === 3. Check if model exists ===
if not os.path.exists(model_path):
    raise FileNotFoundError(f"Model not found at {model_path}. Please check your training save path.")

# === 4. Load the model ===
print("Loading trained PPO model...")
model = PPO.load(
    model_path,
    custom_objects={
        "action_space": env.action_space,
        "observation_space": env.observation_space,
        "device": "cpu"  # Force CPU (PPO with MLP on GPU often inefficient)
    }
)
model.set_env(env)

# === 5. Start verification ===
action_list = []
move_num = 0
print("Starting model verification...")
obs, _ = env.reset()

for step in range(500000):
    action, _ = model.predict(obs, deterministic=True)
    obs, reward, done, truncated, info = env.step(action)
    # env.velocity_set_num = 0
    # env.position_set_num = 0
    # world_move_x = action[0] * 2.0
    # world_move_y = action[1] * 2.0
    # world_move_z = (action[2] + 1.0) * 0.45 + 0.3
    if done is True:
        env.reset()
    # print(f"[Step {step}] Reward: {reward:.2f}")
    # if not done:
    #     action_list.append([world_move_x, world_move_y, world_move_z])
    # if done:
    #     print("[Info] Episode finished. Resetting environment.")
    #     obs, _ = env.reset()
    #     move_num += 1
    # if move_num ==3:
    #     action_array = np.vstack(action_list)
    #     np.savetxt('Action_list.txt', action_array, fmt='%.4f',delimiter=',')

# === 6. Shutdown ===
env.close()
simulation_app.close()

print("Verification complete!")
