"""
Simulate quadrotor
"""

import numpy as np
from math import sin, cos, pi
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt
import importlib

from quad_sim import simulate_quadrotor

# Need to reload the module to use the latest code
import quadrotor
importlib.reload(quadrotor)
from quadrotor import Quadrotor

"""
Load in the animation function
"""
import create_animation
importlib.reload(create_animation)
from create_animation import create_animation

# Weights of LQR cost
R = np.eye(2);
Q = np.diag([10, 10, 1, 1, 1, 1]);
Qf = Q;

# End time of the simulation
tf = 10;

# Construct our quadrotor controller 
quadrotor = Quadrotor(Q, R, Qf);

# Set quadrotor's initial state and simulate
x0 = np.array([0.5, 0.5, 0, 1, 1, 0])
x, u, t = simulate_quadrotor(x0, tf, quadrotor)

anim, fig = create_animation(x, tf)
plt.close()
anim
