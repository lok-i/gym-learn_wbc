import gym
import matplotlib.pyplot as plt
import os
from stable_baselines.common.policies import MlpPolicy
from stable_baselines.common.vec_env import DummyVecEnv
from stable_baselines.common.env_checker import check_env
from stable_baselines import PPO2
import time 
from test_params import *
cwd = os.getcwd()

'''
model name ,target,no of epidodes,
no of steps are all defined in
test_params.py file
'''

env = gym.make('gym_learn_wbc:learn_wbc-v0',
	           learning_steps=no_of_sim_steps,
	           target_velocity=target_velocity,
	           render = True
	           )

#check_env(env, warn=True)


model = PPO2.load(cwd+'/models/'+agent_name+'.zip')



# no of episodes to test
for i_episode in range(10):
	print('\n\nEPISODE_:',i_episode)
	state = env.reset()
	time.sleep(0.001)
	action,_ = model.predict(state)
	state,reward,done,_ = env.step(action)
	print("Omega_predicted:",action[0]*env.omega_range[1])
	print("Radius_predicted:",(action[1]+1)*env.radius_range[1]/2.0)
	print('Reward:\n',reward,"\n")

	
	
	
		
env.close()
