import gym
import matplotlib.pyplot as plt
import os
from stable_baselines.common.policies import MlpPolicy
from stable_baselines.common.vec_env import DummyVecEnv
from stable_baselines import PPO2
from test_params import *



cwd = os.getcwd()
'''
model name ,target,no of epidodes,
no of steps are all defined in
test_params.py file
'''

env = gym.make('gym_learn_wbc:learn_wbc-v0',
	           learning_steps=no_of_sim_steps,
	           target_velocity=target_velocity
	           )
'''
model = PPO2(MlpPolicy,
			 env,
			 verbose=1,
			 tensorboard_log=cwd+'/tf_logs/')

'''
'''
model = PPO2(MlpPolicy,
			 env,
			 gamma=0.95,
			 learning_rate = 0.000001,
			 n_steps=no_of_episodes,
			 noptepochs=10,
			 nminibatches=5,
			 verbose=1,
			 tensorboard_log=cwd+'/tf_logs/')
'''

model = PPO2(MlpPolicy,
			 env,
			 verbose = 1,
			 n_steps=12,
			 tensorboard_log=cwd+'/tf_logs/')

env.reset()

model.learn(total_timesteps=10000,
				 log_interval=1,
				 tb_log_name=agent_name)
model.save(save_path=cwd+'/models/'+agent_name+'.zip')


