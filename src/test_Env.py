import gym
from time import sleep
import numpy as np
initial_base = [0,0,0.6]
no_of_learning_steps = 5
env = gym.make('gym_learn_wbc:learn_wbc-v0',
	           learning_steps=no_of_learning_steps,
	           )

env.step([1,-1])

