#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Sep 28 09:42:54 2024

@author: erik
"""
# Motor speed control simulation. P = 100/(s+10), C = 0.1+1/s 
# Physics run at Ts, controller runs at Tc 

import numpy as np 
import matplotlib.pyplot as plt 
# Ts is the plant step size (physics sim) 
# Tc is the controller step size (u-controller implementation); must be integer multiple of Ts 

Tphys = 0.001
Tc_multiplier = 5
Tc = Tphys*Tc_multiplier
t = np.arange(0, 1, Tphys) 
x = np.zeros((len(t), 1)) # x is motor speed 
# setpoint to track 
ref = np.zeros((len(t), 1)) 
ref[0:len(t)//2, :] = 1 
# controller class 
class PI: 
    def __init__ (self, Kp=1, Ki=1, Tc=0.01, initial_condition=0): 
        self.Kp = Kp 
        self.Ki = Ki 
        self.Tc = Tc 
        self.integrator_state = initial_condition 
        
    def update(self, r, y): 
        error = r - y 
        self.integrator_state = self.integrator_state + self.Tc*error 
        return self.Kp*error + self.Ki*self.integrator_state 

controller = PI(0.1, 1, Tc) 
# Run the sim 
for k in range(len(t) - 1): 
    if (k % Tc_multiplier == 0): 
        # update for the controller 
        y = x[k, :] 
        r = ref[k, :] 
        v = controller.update(r, y) 
    # update for the plant (physics) 
    f = -10*x[k, :] + 100*v 
    x[k + 1, :] = x[k, :] + Tphys*(f) 
# plot 
plt.figure() 
plt.plot(t, x, 'b-', linewidth=2, label='y(t)') 
plt.plot(t, ref, 'r--', label='r(t)') 

plt.xlabel('Time') 

plt.show() 