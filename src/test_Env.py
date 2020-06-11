import gym
from time import sleep
import numpy as np
initial_base = [0,0,0.6]
no_of_learning_steps = 100
env = gym.make('gym_learn_wbc:learn_wbc-v0',
	           learning_steps=no_of_learning_steps,
	           target_velocity=[0.5,0.0]
	           )

#env.step([-0.5,0])

def test_env():
	for i_episode in range(5):
		print('\n\nEPISODE_:',i_episode)
		state = env.reset()
		print("initial_state:",state)
		#sleep(0.01)
		action =[-0.3,-0.42]
		state,reward,done,_ = env.step(action)
		print('State:\n',state,"\n")
		print('Reward:\n',reward,"\n")
		print('done:\n',done,"\n")
		#if done:
			#break
	env.close()

test_env()