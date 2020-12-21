#!/usr/bin/python3
# RUN ME
# This file gives the initial condition to the scipy.integrat.odeint function 
# and plots the resulting state outputs at each time step in an animation and
# on a plot that compares the actual output with the reference input

import numpy as np
import scipy
import sys
sys.path.append('..')  # add parent directory
import matplotlib.pyplot as plt
import control
import pendulumParam as P
from kalmanFilter import kalmanFilter
from pendulumNonlinearDynamics import Pendulum
from pendulumAnimation import pendulumAn
from plotDataZ import plotData      
from signalGenerator import signalGen
from pendulumParam import A, B, C, maxForce

#Compute controller
Q = np.array( [[100, 0, 0,   0],
                [0,  10, 0,   0],
                [0,   0, 100, 0],
                [0,   0,  0, 10]])
R = np.array([[50]])
# Solve ARE (Algebraic Ricatti Equation)
S = scipy.linalg.solve_continuous_are(A, B, Q, R)
# Find Kr: the following line means R^-1 times B^T times S
Kr = np.linalg.inv(R).dot(B.transpose().dot(S))
#poles = np.array([-1.9, -2, -2.1, -2.5])
#Kr = control.place(A, B, poles)
#print("Kr=",Kr)
#print(type(Kr))

#Initialize and rename for convenience
ref = signalGen(amplitude=.5, frequency=0.05, y_offset=0) 
pendulum = Pendulum(param=P)

states = [np.array([[P.z0], [P.zdot0], [P.theta0], [P.thetadot0]])]
#states[0][1,0] += 0.5 # start out in the wrong place
#print("states=",states)
states_est = [states[0]]
mu = np.array([[P.z0], [P.zdot0], [P.theta0], [P.thetadot0]])
sigma = np.eye(4)*0.00001
u=0

#performs very simple first order integration 
length = int((P.t_end-P.t_start)/P.Ts)              #The number of time steps over the time interval
t_array = np.linspace(P.t_start, P.t_end, length)   #The time vector for integration.
dt=t_array[1] - t_array[0]

#O = control.obsv(A,C) # A and C are imported from pendulumParam.py
#yn = ["is not", "is"]
#RO = np.linalg.matrix_rank(O)
#print("The observability matrix {} observable. Its rank is {}".format(yn[RO==4],RO)) 
#print(O)

for t in t_array[:-1]:
    des_state = np.array([[ref.square(t)[0]], [0.0], [0.0], [0.0]])
    #print("des_state=",des_state)
    #print(type(des_state))
    old_state=states[-1]
    #print("states=",states)
    #print(type(states))

    #Update controller and sensors every <T_update> seconds
    if (t % P.T_update) < dt:
        u=np.matmul(Kr,des_state-mu)
        #u = float(u)
        # equivalently, u=-Kr.dot(mu-des_state)
        #print("u=",u)
        #print(type(u))
        # Maximum actuator force
        if u > maxForce:
            u = maxForce
        elif u < -maxForce:
            u = -maxForce
        y_kf = P.C.dot(old_state)
        mu,sigma = kalmanFilter(mu,sigma,u,y_kf)
    #print("old_state=",old_state)
    #print(type(old_state))
    #print("Output from cartpendfunc:\n",pendulum.cartpendfunc(old_state,u))
    new_state=old_state + np.array(pendulum.cartpendfunc(old_state,u)) * dt
    #print("new_state=",new_state)
    #print(type(new_state))
    
    #Arrays for debugging
    states_est.append(mu)
    states.append(new_state)

#animation
plt.close('all')
animation = pendulumAn()

#Recast arrays for plotting
states = np.array(states)
states_est = np.array(states_est)

i = 0
reference = np.zeros((length,1))
while i < len(t_array):  
    #calculate the reference value
    t_curr = t_array[i]                 #Current time step
    z_ref = ref.square(t_curr)          #Get cart z location
    reference[i:(i+100)] = z_ref        #Which reference value is used

    #update animation and data plots
    animation.drawPendulum(states[i,:]) #Animate
    plt.pause(0.001)                    #Pause while plot updates
    i = i + 100                         #speeds up the simulation by not plotting all the points
 
# Plot how closely the actual performance of the pendulum on a cart matched the desired performance
dataPlot = plotData()                       #initializes plot
dataPlot.Plot(t_array, reference, states_est, 1) #plot the estimate
dataPlot.Plot(t_array, reference, states, 2)     #plots the data
