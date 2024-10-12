#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Sep 28 11:54:01 2024

@author: erik
"""

# RLC Simulation. P = 1/(s^2 + s + 1), C = Kp + Ki/s + Kd*s
# Physics run at Ts, controller runs at Tc 

import numpy as np 
import math 
import matplotlib.pyplot as plt 
from PID import PID
# Ts is the plant step size (physics sim) 
# Tc is the controller step size (u-controller implementation); must be integer multiple of Ts 
# Kp = 100.1
# Ki = 120.24
# Kd = 19.2
# Tf = .0047666
#from HW2 solutions: 
Kp = 93
Ki = 108
Kd = 18.49
Tf = .004959
Tphys = 200e-6 #physics time [sec] step
Tc_multiplier = 5;
Tc = Tphys*Tc_multiplier; 

t = np.arange(0, 2, Tphys) 
vc = np.zeros((len(t), 1)) # x is motor speed 
# setpoint to track 
ref = np.zeros((len(t), 1)) 
ref[0:len(t)//2, :] = 1 
z_prev = 0
z=0
fig, ax = plt.subplots()   
for kt in [.8, 1, 1.3, 2, 5]:   
    controller = PID(Kp=Kp, Ki=Ki, Kd=Kd, Tf=Tf, Tc=Tc, Kt=kt,
                      sat_max=10, sat_min=-10)
    # Run the sim 
    for k in range(len(t) -1): 
        if (k % Tc_multiplier == 0): 
            # update for the controller 
            y = vc[k, :] 
            r = ref[k, :] 
            v = controller.update(r, y) 
        # update for the plant (physics) 
        #this is plant from HW2 Q3 (the RLC circuit with R=L=C=1)
           
        zdot =  -vc[k,:]-z + v
        xdot = z
        z = z +zdot*Tphys
        vc[k+1,:] = vc[k,:] + Tphys*xdot 
    
# plot 

    ax.plot(t, vc, linewidth=2, label=f'kt = {kt:.1f}') 
ax.plot(t, ref, 'r--', label='r(t)') 
ax.set_ylim(-.4, 1.4)
ax.set_xlabel('Time [sec]') 
ax.set_ylabel('Capacitor Voltage [V]')
ax.set_title('Closed Loop RLC Step Response')
ax.legend()
plt.show() 