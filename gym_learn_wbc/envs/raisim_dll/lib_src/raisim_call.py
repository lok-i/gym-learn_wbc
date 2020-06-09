'''
file to test the functions
in raisim dll created
'''


from ctypes import *
import random
import numpy as np
stoch_raisim_dll = CDLL("../build/libstoch3_raisim.so")
float_array_3 = c_float*3
initial_base = float_array_3()
initial_base[0] = 0
initial_base[1] = 0
initial_base[2] = 0.6


stoch_raisim_dll._init_ViSsetup(c_bool(True))
'''
second argument of init stoch:
0 - PD CONTROL
1 - FORCE TORQUE CONTROL
'''
stoch_raisim_dll._init_stoch(initial_base,c_int(0))




stoch_raisim_dll._sim.restype = None
# omega,radius
stoch_raisim_dll._sim.argtypes = [c_long,c_double,c_double,c_bool]

stoch_raisim_dll._reset.restype = None
stoch_raisim_dll._reset.argtypes = [c_float*3]
render = c_bool(True)
no_of_steps = c_long(100)
omega = c_double(-3)
radius = c_double(0.06)

for i in range(10):
	stoch_raisim_dll._reset(initial_base)
	stoch_raisim_dll._sim(no_of_steps,omega,radius,render)
