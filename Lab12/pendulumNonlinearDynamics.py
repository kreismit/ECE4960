# This file runs the nonlinear pendulum dynamics 
# You can add process noise here (line 60)

import numpy as np
from pendulumParam import maxVel, threshold

class Pendulum:

    def __init__(self, m1=None, m2=None, ell=None, b=None, g=None,param=None):
        
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
            
    def cartpendfunc(self,x,u):
        '''Nonlinear dynamics'''
    
        #unpack the state (named x in lectures)
        z, zdot, theta, thetadot = x
        theta = theta+np.pi #treat 0 as upward
        #print("theta=",theta)
        #print(type(theta))
        
        
        #simplifications for the calculations - constants
        Sy = np.sin(theta)
        Cy = np.cos(theta)
        #print("Sy=",Sy)
        #print(type(Sy))
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
        ydot1 = (1.0/D)*(-self.m1*self.m1*self.ell*self.ell*self.g*Cy*Sy + self.m1*self.ell*self.ell*(self.m1*self.ell*thetadot*thetadot*Sy - self.b*zdot)) + self.m1*self.ell*self.ell*(1.0/D)*u
        if zdot > maxVel: # maximum velocity to the right
            ydot1 = 0 # can't accelerate any more
        elif zdot < -maxVel: # maximum velocity to the left
            ydot1 = 0 # can't accelerate any more
        ydot2 = thetadot
        ydot3 = (1.0/D)*((self.m1+self.m2)*self.m1*self.g*self.ell*Sy    - self.m1*self.ell*Cy*(self.m1*self.ell*thetadot*thetadot*Sy - self.b*zdot)) - self.m1*self.ell*Cy*(1.0/D)*u
        #print("u=",u)
        #print(type(u))
        #dydt = [ydot0, ydot1, ydot2, ydot3]
        ydot0 = float(ydot0)
        ydot1 = float(ydot1)
        ydot2 = float(ydot2)
        ydot3 = float(ydot3)
        #dydt = np.array([[ydot0], [ydot1], [ydot2], [ydot3]]) # no noise
        #print("dydt=",dydt)
        #with process noise:
        dydt = np.array([[ydot0 + np.random.randn()*0.5], # 1/2 m noise
        [ydot1 + np.random.randn()*0.25],    # 1/4 m/s noise
        [ydot2 + np.random.randn()*0.174], # 10 deg noise
        [ydot3 + np.random.randn()*0.087]]) # 5 deg/s
        return dydt


