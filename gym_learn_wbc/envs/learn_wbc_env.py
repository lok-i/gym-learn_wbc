import gym
from gym import error, spaces, utils
import numpy as np
from gym.utils import seeding
from ctypes import *

raisim_dll = CDLL("../gym_learn_wbc/envs/raisim_dll/build/libstoch3_raisim.so")
def init_raisim_dll_functions():
	raisim_dll._sim.restype = None
	raisim_dll._sim.argtypes = [c_double*10,c_long,c_double,c_double,c_bool,c_float*2]
	raisim_dll._reset.restype = None
	raisim_dll._reset.argtypes = [c_float*3]


class Learn_wbcEnv(gym.Env):
  metadata = {'render.modes': ['human']}

  def __init__(self,learning_steps,target_velocity,render=False):
    print("init")
    # fixed properties
    #self.mass = mass
    #self.Rotational_inertia = rot_inertia
    #self.friction_coeff = fric_coeff

    # logitudinal velocity and lateral velocity
    self.target_velocity = [target_velocity[0],target_velocity[1]] 
    self.render = render

    self.initial_base = (c_float*3)()
    self.initial_base[0] = 0
    self.initial_base[1] = 0
    self.initial_base[2] = 0.6

    self.learning_steps = learning_steps
    self.omega_range = [-5,5]
    self.radius_range = [0.1,0.21]
    
    self.avg_velocity_limits = [-1.5,1.5]
    '''
    the max distance between either vmin or vmax
    is checked and is initialized as the max allowed 
    error to normalize.

    '''
    vtx_vmin = np.abs(self.target_velocity[0] - self.avg_velocity_limits[0])
    vtx_vmax = np.abs(self.target_velocity[0] - self.avg_velocity_limits[1])
    
    vty_vmin = np.abs(self.target_velocity[1] - self.avg_velocity_limits[0])
    vty_vmax = np.abs(self.target_velocity[1] - self.avg_velocity_limits[1])

    if(vtx_vmin <= vtx_vmax):
        if(vty_vmin <= vty_vmax):
            self.avg_velocity_error_limits = [0,vtx_vmax,0,vty_vmax]
        else:
            self.avg_velocity_error_limits = [0,vtx_vmax,0,vty_vmin]
    else:
        if(vty_vmin <= vty_vmax):
            self.avg_velocity_error_limits = [0,vtx_vmin,0,vty_vmax]
        else:
            self.avg_velocity_error_limits = [0,vtx_vmin,0,vty_vmin]

    self.avg_ang_velocity_limits = [-0.1,0.1]
    self.action_space = spaces.Box(low=-1.0, high=1.0,shape=(2,))
    '''
    state space : target vel,avg_vel_error,avg_vel,rpy,r_dot p_dot y_dot
    '''
    self.observation_space = spaces.Box(low=-1.0, high=1.0,shape=(10,))
    
    raisim_dll._init_ViSsetup(c_bool(True))
    raisim_dll._init_stoch(self.initial_base,c_int(0))
    init_raisim_dll_functions()


  def step(self,action):
    '''
    1.at every step the agent will predict a configuration of the controller
    2.a simulation rollout in raisim tracking a velocity target
    3.calculate reward based on the roll out
    3.return reward,state,wether the model has reached saturation(ie learnt)
    '''

    omega = c_double(0.5*( (self.omega_range[1]-self.omega_range[0])*action[0]
                          +(self.omega_range[1]+self.omega_range[0])))

    radius = c_double(0.5*((self.radius_range[1]-self.radius_range[0])*action[1]
                          +(self.radius_range[1]+self.radius_range[0])))
    state = (c_double*10)()
    target_velocity = (c_float*2)()
    target_velocity[0] = self.target_velocity[0];
    target_velocity[1] = self.target_velocity[1];

    raisim_dll._sim(state,self.learning_steps,
                    omega,radius,c_bool(self.render),
                    target_velocity)

    state = np.array(state)
    print("avgVx:",state[0],"avgVy:",state[1])
    reward = self.calculate_reward(state[0:2])
    #print("reward:",reward)

    #clipping
    state[0:2] = np.clip(state[0:2],self.avg_velocity_limits[0],self.avg_velocity_limits[1])
    state[2:3] = np.clip(state[2:3],-1*self.avg_velocity_error_limits[1],self.avg_velocity_error_limits[1])
    state[3:4] = np.clip(state[3:4],-1*self.avg_velocity_error_limits[3],self.avg_velocity_error_limits[3])
    state[4:7] = np.clip(state[4:7],-np.pi/2,np.pi/2)
    state[7:10]= np.clip(state[7:10],self.avg_ang_velocity_limits[0],self.avg_ang_velocity_limits[1])

    #normalizing _ only for symmetrical limits 
    state[0:2] = (1/self.avg_velocity_limits[1])*state[0:2]
    state[2:3] = (1/self.avg_velocity_error_limits[1])*state[2:3]
    state[3:4] = (1/self.avg_velocity_error_limits[3])*state[3:4]
    state[4:7] = (2/np.pi)*state[4:7]
    state[7:10] = (1/self.avg_ang_velocity_limits[1])*state[7:10]

    return state,reward,True,{}
  
  def calculate_reward(self,avg_vel):

    weight_matrix = np.array([0.5,0.5])
    exp_weight = [-6,-6]
    abs_error = np.absolute(avg_vel - self.target_velocity)

    exponential_error = np.exp(exp_weight[0]*abs_error[0]+exp_weight[1]*abs_error[1])

    return(exponential_error)




    
  def reset(self):
    '''
    1.reset simulation parameters
    '''
    raisim_dll._reset(self.initial_base)
    radius_zero_action =( self.radius_range[0] + self.radius_range[1] )/(self.radius_range[0] - self.radius_range[1])
    omega_zero_action =( self.omega_range[0] + self.omega_range[1] )/(self.omega_range[0] - self.omega_range[1])
    initial_state = self.step([omega_zero_action,radius_zero_action])[0]
    #print("reset")
    return initial_state
    
  def render(self, mode='human'):
    '''
    visulaize a frame (need to chk if this function is required)
    print("render")
    '''
  def close(self):
    '''
    kill env 
    '''
    raisim_dll._close()
    print("close")