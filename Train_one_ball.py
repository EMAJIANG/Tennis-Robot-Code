from isaacsim.simulation_app import SimulationApp
import os
CONFIG = {"renderer": "RayTracedLighting", "headless": True}

simulation_app = SimulationApp(launch_config=CONFIG)

import time
time.sleep(2)
import sys
import os
import numpy as np
from stable_baselines3 import PPO
from stable_baselines3.common.env_checker import check_env
from stable_baselines3.common.logger import configure
from stable_baselines3.common.callbacks import BaseCallback, EvalCallback
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.vec_env import SubprocVecEnv
from tqdm import tqdm
import Train.TR_one_ball_env
# from Train.TR_one_ball_env import IsaacOneBallEnv
import smtplib
from email.mime.text import MIMEText
from datetime import datetime
import time

# Define current timestamp once
now_time = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
log_dir = f"./logs_tensorboard_{now_time}/"
os.makedirs(log_dir, exist_ok=True)
progress_file = f"{log_dir}/training_progress_{now_time}.log"

# TensorBoard callback
class TensorboardCallback(BaseCallback):
    def __init__(self, verbose=0):
        super(TensorboardCallback, self).__init__(verbose)

    def _on_step(self) -> bool:
        return True

# Email sending function
def send_email(final_reward, end_time):
    sender_email = "1463104117@qq.com"
    sender_password = "rmcnafbblpsqhajd"
    receiver_email = "laijiang2425@gmail.com"

    subject = f"Training Completed at {now_time}"
    body = f"Final Reward: {final_reward}\nEnd Time: {end_time}"

    msg = MIMEText(body)
    msg['Subject'] = subject
    msg['From'] = sender_email
    msg['To'] = receiver_email

    try:
        with smtplib.SMTP_SSL('smtp.qq.com', 465) as server:
            server.login(sender_email, sender_password)
            server.sendmail(sender_email, receiver_email, msg.as_string())
        print("✅ Email sent successfully!")
    except Exception as e:
        print(f"❌ Failed to send email: {e}")

# Update training progress
def update_progress(current_steps, total_steps):
    with open(progress_file, 'w') as f:
        percent = 100.0 * current_steps / total_steps
        f.write(f"Progress: {current_steps}/{total_steps} ({percent:.2f}%)\n")
        f.write(f"Current Time: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")

# Main training function
def main():
    # env = make_vec_env(IsaacOneBallEnv(), n_envs = 2, vec_env_cls = SubprocVecEnv)
    env = Train.TR_one_ball_env.IsaacOneBallEnv()
    env.reset_mode = False
    check_env(env)

    new_logger = configure(log_dir, ["stdout", "tensorboard"])

    # model = PPO(
    #     policy="MlpPolicy",
    #     env=env,
    #     learning_rate=1.5e-4,         # 默认学习率即可（探索用）
    #     n_steps=8192,               # 短一点，提高更新频率
    #     batch_size=1024,              # 保持稳定
    #     gae_lambda=0.95,            # 一般保留不变
    #     gamma=0.99,                 # 缩短未来奖励影响，促进快速试错
    #     clip_range=0.2,             # 默认即可
    #     ent_coef=0.02,              # 增加策略熵奖励
    #     vf_coef=0.4,
    #     max_grad_norm=0.5,
    #     verbose=1
    # )
    model = PPO.load("./logs_tensorboard_2025-06-16_16-43-16/best_model/best_model.zip", 
                     gamma=0.99,env=env,batch_size=512,learning_rate=2.5e-4, device = "cpu")

    model.set_logger(new_logger)

    total_timesteps = 1024*128
    batch_size = 1024
    timesteps_per_iteration = batch_size

    eval_callback = EvalCallback(
        env,
        best_model_save_path=os.path.join(log_dir, "best_model"),
        log_path=log_dir,
        eval_freq=2000,
        deterministic=True,
        render=False
    )

    tensorboard_callback = TensorboardCallback()

    pbar = tqdm(total=total_timesteps, desc="Training Progress", unit="steps")

    final_reward = None

    while pbar.n < total_timesteps:
        model.learn(
            total_timesteps=timesteps_per_iteration,
            reset_num_timesteps=False,
            callback=[eval_callback, tensorboard_callback]
        )
        pbar.update(timesteps_per_iteration)
        update_progress(pbar.n, total_timesteps)
        if 'rollout/ep_rew_mean' in model.logger.name_to_value:
            final_reward = model.logger.name_to_value['rollout/ep_rew_mean']

    pbar.close()

    model.save(os.path.join(log_dir, "ppo_isaac_one_ball_final"))

    end_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

    if final_reward is not None:
        send_email(final_reward, end_time)

    env.close()
    simulation_app.close()

if __name__ == "__main__":
    main()