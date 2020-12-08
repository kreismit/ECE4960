# This file contains physical parameters for the cart and the system state space equations

import numpy as np
import sys
sys.path.append('..')  # add parent directory
import scipy.linalg
from scipy import signal

#Physical parameters of the inverted pendulum known to the controller
m1 = 0.03   # Mass of the pendulum [kg]
m1Real = 0.03
m2 = .610   # Mass of the cart [kg]
m2Real = 0.55
ell = 1.21  # Length of the rod [m]
g = -9.81   # Gravity, [m/s^2]
b = 0.78   # Damping coefficient [Ns]
bReal = 0.85
maxVel = 3.5 # maximum linear speed of robot (m/s)
threshold = 0.05 # minimum linear speed of robot (m/s)
maxForce = 8.46 # maximum actuator force (N)

#parameters for animation
w = 0.15      # Width of the cart [m]
h = 0.07      # Height of the cart [m]
gap = 0.005   # Gap between the cart and x-axis [m]
radius = 0.08 # Radius of circular part of pendulum [m]

#Simulation Parameters
t_start = 0.0   # Start time of simulation [s]
t_end = 30.0    # End time of simulation [s]
Ts = 0.001      # sample time for simulation [s]
t_plot = 0.5    # Animation update rate [s]

# Sensor Noise Standard Deviations

zNoise = 0.0
zDotNoise = 0.0
thetaNoise = 0.0*np.pi/180 # 1° standard deviation
thetaDotNoise = 0.0*np.pi/180 #0.1°/s std. dev.

#Initial Conditions
z0 = 0.01               # [m]
zdot0 = 0.0             # [m/s]
theta0 = np.pi +.1      # [rad], starts upright
thetadot0 = 0.0         # [rads/s]

####################################################
#                 State Space
####################################################
#xdot = A*x + B*u
#y = C*x
#x = [z, zdot, theta, thetadot]

A = np.matrix([[0.0, 1.0, 0.0, 0.0],
            [0.0, -b/m2, -m1*g/m2, 0.0],
            [0.0, 0.0, 0.0, 1.0],
            [0.0, -b/(m2*ell), -(m1+m2)*g/(m2*ell), 0.0]])
AReal = np.matrix([[0.0, 1.0, 0.0, 0.0],
            [0.0, -bReal/m2Real,   -m1Real*g/m2Real,                0.0],
            [0.0, 0.0,              0.0,                            1.0],
            [0.0, -bReal/(m2*ell), -(m1Real+m2Real)*g/(m2Real*ell), 0.0]])
B = np.array([[0.0], [1.0/m2], [0.0], [1.0/(m2*ell)]])
BReal = np.array([[0.0], [1.0/m2Real], [0.0], [1.0/(m2Real*ell)]])

C = np.matrix([[1.0, 0.0, 0.0, 0.0],
               [0.0, 1.0, 0.0, 0.0],
               [0.0, 0.0, 1.0, 0.0],
               [0.0, 0.0, 0.0, 1.0]]) 

