# This file contains the scipy.integrate function called in the runSimulation.py file. 
# I also added the force contraint function here and in the kalman filter (when it's being called)

import numpy as np
import scipy
import control
from signalGenerator import signalGen
from pendulumParam import A, B, C, maxVel, threshold

class pendulumCnt:

    def __init__(self, m1=None, m2=None, ell=None, b=None, g=None,
                 Ts=None,angle_limit=None, K=None, param=None, zref=None):
        #Initial state conditions
        
        # Mass of the pendulum, kg
        if not m1 is None:
            self.m1 = m1          
        elif not param is None:
            self.m1 = param.m1
        
        # Mass of the cart, kg
        if not m2 is None:            
            self.m2 = m2
        elif not param is None:
            self.m2 = param.m2
       
         # Length of the rod, m
        if not ell is None:    
            self.ell = ell
        elif not param is None:
            self.ell = param.ell       
            
        # Damping coefficient, Ns 
        if not b is None:
            self.b = b                   
        elif not param is None:
            self.b = param.b            
                
        # Gravity constant
        if not g is None:
            self.g = g            
        elif not param is None:
            self.g = param.g          
            
        # sample rate of controller
        if not Ts is None:    
            self.Ts = Ts          
        elif not param is None:
            self.Ts = param.Ts        
            
        if not K is None:
            self.K = K
        elif not param is None:
            if 'K' in param.__dict__.keys():
                self.K = param.K
            else:
                self.K= None
        else:
            self.K = None
            
        self.angle_limit = np.pi*2.0
        
        if not zref is None:
            self.zref=zref
        else:
            self.reference = signalGen(amplitude=.5, frequency=0.05, y_offset=0)
            self.zref= self.reference.square

    ####################################################
    #               scipy.integrate
    ####################################################
    def cartpendfunc(self,y,t):
        #unpack the state
        z, zdot, theta, thetadot = y

        #don't allow theta to exceed 2pi
        #need this so that equivalnent angles don't cause a difference in 
        #in the reference input
        theta = self.limitTheta(theta)

        #Get reference inputs from signal generators
        zref = self.zref(t)      #control cart z location
        
        #calculting the new control force
        curr_state = np.array([[z], [zdot], [theta], [thetadot]])   #current state
        des_state = np.array([zref, [0.0], [np.pi], [0.0]])         #desired state

        #Feedback control. If there's no gain it assigns the control to be 0
        '''
        Compute the input self.u
        Your Code Here
        '''
        Q = np.matrix( [[100, 0, 0,   0],
                        [0,  10, 0,   0],
                        [0,   0, 100, 0],
                        [0,   0,  0, 10]])
        R = np.matrix([50])
        # Solve ARE (Algebraic Ricatti Equation)
        S = scipy.linalg.solve_continuous_are(A, B, Q, R)
        # Find Kr: the following line means R^-1 times B^T times S
        Kr = np.linalg.inv(R).dot(B.transpose().dot(S))
        #poles = np.array([-1.9, -2, -2.1, -2.5])
        #Kr = control.place(A, B, poles)
        self.u = np.matmul(Kr, des_state - curr_state)

        #simplifications for the calculations - constants
        Sy = np.sin(theta)
        Cy = np.cos(theta)
        D = self.m1*self.ell*self.ell*(self.m2+self.m1*(1.0-Cy*Cy))

        #calculating state values at the current time step
        if zdot > maxVel: # maximum velocity to the right
            ydot0 = maxVel
        elif zdot < -maxVel: # maximum velocity to the left
            ydot0 = -maxVel
        elif zdot > threshold or zdot < -threshold: # deadband
            ydot0 = zdot
        else: # can't move slower than a certain speed
            ydot0 = 0
        ydot1 = (1.0/D)*(-self.m1*self.m1*self.ell*self.ell*self.g*Cy*Sy + self.m1*self.ell*self.ell*(self.m1*self.ell*thetadot*thetadot*Sy - self.b*zdot)) + self.m1*self.ell*self.ell*(1.0/D)*self.u
        if zdot > maxVel: # maximum velocity to the right
            ydot1 = 0 # can't accelerate any more
        elif zdot < -maxVel: # maximum velocity to the left
            ydot1 = 0 # can't accelerate any more
        ydot2 = thetadot
        ydot3 = (1.0/D)*((self.m1+self.m2)*self.m1*self.g*self.ell*Sy    - self.m1*self.ell*Cy*      (self.m1*self.ell*thetadot*thetadot*Sy - self.b*zdot)) - self.m1*self.ell*Cy*(1.0/D)*self.u
        dydt = [ydot0, ydot1, ydot2, ydot3]
        return dydt


    ####################################################
    #              Extra Functions
    ####################################################
    def limitTheta(self, angle):
        if abs(angle) > self.angle_limit:
            rem = (abs(angle) % self.angle_limit)*np.sign(angle)
        elif abs(angle) <= self.angle_limit:
            rem = angle
        return rem
    
