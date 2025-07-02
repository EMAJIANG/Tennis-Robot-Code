from algos.comoppo.comoppo import CoMOPPO
from Train.TR_one_ball_env import IsaacOneBallEnv

env = IsaacOneBallEnv()
model = CoMOPPO(env, 
                task_cfg="experiments/oneball_reward_stages.yaml", 
                algo_cfg="experiments/comoppo_hyper.yaml")

model.learn(total_timesteps=1_000_000)
model.save("comoppo_stage_model")
