Isaac Lab Training
https://youtu.be/o9Bym5mOl2k?si=5StA813SR1jcSOGM
Go to /home/lai-jiang/IsaacLab/IsaacLab/source/standalone/workflows/sb3, 
do play
python3 play.py --task Custom-Wheeled-Quadruped-v0 --num_envs 8 --use_last_checkpoint
do train
python3 train.py --task Custom-Wheeled-Quadruped-v0 --num_envs 8

The source files should be on /home/lai-jiang/IsaacLab/IsaacLab/source/extensions/omni.isaac.lab_tasks/omni/isaac/lab_tasks/manager_based/classic/wheeled_quadruqed/wheeled_quadruped_env_cfg.py

Making Tennis Robot file base on wheeled_quadruped_env_cfg, define the movement and reward functions.

Change /home/lai-jiang/IsaacLab/IsaacLab/source/extensions/omni.isaac.lab_tasks/omni/isaac/lab_tasks/manager_based/classic/wheeled_quadruqed/__init__.py 
gym.register(
    id="Tennis_Robot-v0",
    entry_point="omni.isaac.lab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": WheeldQudrupedEnvCfg,
        "rl_games_cfg_entry_point": f"{agents.__name__}:rl_games_ppo_cfg.yaml",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:WheeledQuadrupedPPORunnerCfg",
        "skrl_cfg_entry_point": f"{agents.__name__}:skrl_ppo_cfg.yaml",
        "sb3_cfg_entry_point": f"{agents.__name__}:sb3_ppo_cfg.yaml",
    },
)
