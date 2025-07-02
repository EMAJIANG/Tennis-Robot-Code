# Import SimulationApp here, it will be used in make_env and for eval_env
from isaacsim import SimulationApp
main_process_simulation_app = SimulationApp({"renderer": "RaytracedLighting", "headless": True})
import os
import numpy as np
from stable_baselines3 import PPO
from stable_baselines3.common.env_checker import check_env
from stable_baselines3.common.logger import configure
from stable_baselines3.common.callbacks import BaseCallback, EvalCallback
from stable_baselines3.common.vec_env import SubprocVecEnv # Import SubprocVecEnv
from stable_baselines3.common.env_util import make_vec_env # Helpful utility


# Assuming Train.TR_one_ball_env contains IsaacOneBallEnv
from Train.TR_one_ball_env import IsaacOneBallEnv
from tqdm import tqdm
import smtplib
from email.mime.text import MIMEText
from datetime import datetime
import time

# Define current timestamp once
now_time = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
log_dir = f"./logs_tensorboard_{now_time}/"
os.makedirs(log_dir, exist_ok=True)
progress_file = f"{log_dir}/training_progress_{now_time}.log"

# TensorBoard callback (remains the same, though SB3 provides its own TensorBoard logging)
class TensorboardCallback(BaseCallback):
    def __init__(self, verbose=0):
        super(TensorboardCallback, self).__init__(verbose)

    def _on_step(self) -> bool:
        # This callback currently doesn't add custom TB logging beyond what SB3 does.
        # You can add self.logger.record('custom/my_value', value) here if needed.
        return True

# Email sending function (remains the same)
def send_email(final_reward, end_time):
    sender_email = "1463104117@qq.com"
    # IMPORTANT: It's highly recommended to use environment variables or a config file for credentials
    # instead of hardcoding them in the script.
    sender_password = "rmcnafbblpsqhajd" # Consider using getpass or environment variables
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

# Update training progress (remains the same)
def update_progress(current_steps, total_steps):
    with open(progress_file, 'w') as f:
        percent = 100.0 * current_steps / total_steps
        f.write(f"Progress: {current_steps}/{total_steps} ({percent:.2f}%)\n")
        f.write(f"Current Time: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")

# Function to create an environment instance. This will be used by SubprocVecEnv.
def make_env_fn(rank: int, seed: int = 0):
    """
    Utility function for multiprocessed env.
    :param rank: index of the subprocess
    :param seed: RLearN G seed for the environment
    """
    def _init():
        # CRITICAL: Each subprocess needs its own SimulationApp.
        # This assumes IsaacOneBallEnv will use the SimulationApp created in its process.
        # And that IsaacOneBallEnv.close() will handle closing this sim_app.
        # Add any specific Isaac Sim configurations here if needed per environment.
        # e.g., if you have multiple GPUs and want to assign them:
        # sim_config = {"renderer": "RaytracedLighting", "headless": True, "active_gpu": rank % num_available_gpus}
        sim_config = {"renderer": "RaytracedLighting", "headless": True}
        # The following line creates a SimulationApp instance for this specific process.
        # It's vital that IsaacOneBallEnv is designed to work with this setup, either by
        # detecting and using this instance or by creating its own if needed.
        # Also, ensure that IsaacOneBallEnv's close() method properly shuts down this SimulationApp.
        # If IsaacOneBallEnv internally creates SimulationApp, this line might be redundant
        # or even cause issues if SimulationApp is a strict singleton per process.
        # Test this carefully.
        _ = SimulationApp(sim_config) # Assign to _ if not directly passed to env

        env = IsaacOneBallEnv()
        env.seed(seed + rank)
        return env
    return _init

# Main training function
def main():
    num_threads = 3  # Number of parallel environments (processes)

    # --- Setup for a single environment for check_env and potentially EvalCallback ---
    # This instance is created in the main process.
    # It needs its own SimulationApp.
    print("Initializing SimulationApp for main process (check_env/eval_env)...")
    # This simulation_app is for the main process (for check_env and eval_env)
    # Note: The variable `simulation_app` here is local to `main`, not global to the script.
    
    # Create a single environment instance for check_env
    # This env will use main_process_simulation_app
    print("Creating single env instance for check_env...")
    single_env = IsaacOneBallEnv()
    check_env(single_env)
    print("✅ check_env passed.")
    
    # The single_env can be used for EvalCallback, or you can create a fresh one.
    # If using this single_env for EvalCallback, ensure it's reset appropriately by the callback.
    eval_env = single_env
    # Alternatively, to ensure EvalCallback has a completely fresh env:
    # eval_env = IsaacOneBallEnv() # This would also use main_process_simulation_app

    # --- Setup for vectorized training environments ---
    print(f"Creating {num_threads} parallel environments for training...")
    # Each environment in SubprocVecEnv will be created by make_env_fn in a new process.
    # Each of those processes will initialize its own SimulationApp.
    vec_env = SubprocVecEnv([make_env_fn(i) for i in range(num_threads)], start_method='spawn')

    new_logger = configure(log_dir, ["stdout", "tensorboard"])

    # PPO parameters:
    # n_steps: Number of steps to run for each environment per update.
    # Total samples per update = n_steps * num_threads.
    # Consider adjusting n_steps if you want to keep the total samples per update similar
    # to your single-threaded setup, or if training becomes unstable.
    # Original n_steps = 8192. With 3 threads, this means 8192*3 = 24576 samples per update.
    # This is a large rollout, which is often good for PPO.
    model = PPO(
        policy="MlpPolicy",
        env=vec_env,                # Use the vectorized environment
        learning_rate=2e-4,
        n_steps=8192,               # Per environment
        batch_size=1024,            # Minibatch size for optimization
        gae_lambda=0.95,
        gamma=0.99,
        clip_range=0.2,
        ent_coef=0.02,
        vf_coef=0.4,
        max_grad_norm=0.5,
        verbose=1,
        tensorboard_log=log_dir # SB3 will handle TensorBoard logging
    )

    # model = PPO.load("./logs_tensorboard_2025-05-17_03-41-16/best_model/best_model.zip", env=vec_env) # Load into vec_env
    model.set_logger(new_logger)

    total_timesteps = 200000
    # timesteps_per_iteration determines how many steps model.learn() runs before your manual loop continues.
    # This is different from PPO's n_steps.
    timesteps_per_iteration = 2048 # Keep user's original loop structure

    eval_callback = EvalCallback(
        eval_env, # Use the single, non-vectorized environment for evaluation
        best_model_save_path=os.path.join(log_dir, "best_model"),
        log_path=log_dir,
        eval_freq=max(2000 // num_threads, 1), # Adjust eval_freq based on n_envs or keep as desired
        deterministic=True,
        render=False
    )

    # The custom TensorboardCallback might be redundant if not adding specific metrics,
    # as SB3's logger already handles TensorBoard.
    tensorboard_custom_callback = TensorboardCallback()

    pbar = tqdm(total=total_timesteps, desc="Training Progress", unit="steps")
    final_reward = None
    start_learn_time = time.time()

    # Training loop
    # The number of steps in model.learn is the total steps across all parallel environments.
    current_total_steps = 0
    while current_total_steps < total_timesteps:
        # Calculate remaining steps for this iteration, ensuring we don't exceed total_timesteps
        steps_this_iteration = min(timesteps_per_iteration, total_timesteps - current_total_steps)
        if steps_this_iteration <= 0:
            break

        model.learn(
            total_timesteps=steps_this_iteration, # Steps for this call to learn()
            reset_num_timesteps=False,      # Important for looped learning
            callback=[eval_callback, tensorboard_custom_callback] # Pass callbacks
        )
        current_total_steps += steps_this_iteration # PPO model's num_timesteps tracks total steps
        pbar.update(steps_this_iteration) # Update progress bar by steps done in this learn call
        update_progress(model.num_timesteps, total_timesteps)

        # Check for logged reward (SB3 logs this automatically)
        # Accessing logger directly like this can sometimes be tricky if names change.
        # It's safer to rely on EvalCallback for consistent reward tracking if issues arise.
        if model.logger:
            if 'rollout/ep_rew_mean' in model.logger.name_to_value:
                final_reward = model.logger.name_to_value['rollout/ep_rew_mean']
            elif 'eval/mean_reward' in model.logger.name_to_value: # From EvalCallback
                 final_reward = model.logger.name_to_value['eval/mean_reward']


    pbar.close()
    end_learn_time = time.time()
    print(f"Training loop finished. Total time: {(end_learn_time - start_learn_time)/60:.2f} minutes.")

    model.save(os.path.join(log_dir, "ppo_isaac_one_ball_final"))
    print(f"Final model saved to {os.path.join(log_dir, 'ppo_isaac_one_ball_final')}")

    end_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

    if final_reward is not None:
        send_email(final_reward, end_time)
    else:
        print("⚠️ final_reward was not captured. Email not sent with reward value.")


    print("Closing environments...")
    vec_env.close() # This should trigger close() on all sub-environments, including their SimApps
    # The eval_env (which is single_env) also needs its SimulationApp closed.
    # If IsaacOneBallEnv.close() handles its SimApp, this is fine.
    # Otherwise, we need to explicitly close main_process_simulation_app
    eval_env.close() # Assuming single_env.close() or eval_env.close() handles its SimApp

    # Explicitly close the main process's SimulationApp if not handled by env.close()
    # This is a fallback. Ideally, env.close() should manage its own simulation app.
    # If IsaacOneBallEnv.close() calls its sim_app.close(), this next line is redundant and might error.
    # Test to see if it's needed.
    print("Closing main process SimulationApp...")
    main_process_simulation_app.close()
    print("✅ All closed.")


if __name__ == "__main__":
    # IMPORTANT for SubprocVecEnv, especially on Windows or macOS with 'spawn' start method:
    # The main_logic() must be within this if __name__ == "__main__": block.
    main()