import gym
from gym import error, spaces, utils
from gym.utils import seeding
from ctypes import *

raisim_dll = CDLL("../gym_learn_wbc/envs/raisim_dll/build/libstoch3_raisim.so")
def init_raisim_dll_functions():
	raisim_dll._sim.restype = None
	raisim_dll._sim.argtypes = [c_double*12,c_long,c_double,c_double,c_bool,c_float*2]
	raisim_dll._reset.restype = None
	raisim_dll._reset.argtypes = [c_float*3]


class Learn_wbcEnv(gym.Env):
  metadata = {'render.modes': ['human']}

  def __init__(self,learning_steps):
    print("init")
    # fixed properties
    #self.mass = mass
    #self.Rotational_inertia = rot_inertia
    #self.friction_coeff = fric_coeff
    self.target_velocity = [0,0] # logitudinal velocity and lateral velocity

    self.initial_base = (c_float*3)()
    self.initial_base[0] = 0
    self.initial_base[1] = 0
    self.initial_base[2] = 0.6
    self.learning_steps = learning_steps
    self.omega_range = [-10,10]
    self.radius_range = [0,0.21]
    self.action_space = spaces.Box(low=-1.0, high=1.0,shape=(2,))
    '''
    state space : target vel,avg_vel_error,avg_vel,rpy,r_dot p_dot y_dot
    '''
    self.observation_space = spaces.Box(low=-1.0, high=1.0,shape=(12,))
    
    raisim_dll._init_ViSsetup(c_bool(True))
    raisim_dll._init_stoch(self.initial_base,c_int(0))
    init_raisim_dll_functions()


  def step(self,action):
    '''
    1.at every step the agent will predict a configuration of the wbc
    2.a simulation rollout in raisim tracking a velocity target
    3.calculate reward based on the roll out
    3.return reward,state,wether the model has reached saturation(ie learnt)
    '''

    omega = c_double(action[0]*self.omega_range[1])
    radius = c_double((action[1]+1)*self.radius_range[1]/2.0)
    state = (c_double*12)()
    target_velocity = (c_float*2)()
    target_velocity[0] = 1.0;
    target_velocity[1] = 1.0;
    raisim_dll._sim(state,self.learning_steps,omega,radius,c_bool(True),target_velocity)

    for i in range(12):
    	print("state",i,state[i])
  def reset(self):
    '''
    1.reset simulation parameters
    '''
    print("reset")
  def render(self, mode='human'):
    '''
    visulaize a frame (need to chk if this function is required)
    print("render")
    '''
  def close(self):
    '''
    kill env 
    '''
    print("close")