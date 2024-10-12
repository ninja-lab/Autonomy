#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Oct  7 15:01:58 2024

@author: erik
"""
import pandas as pd 
import matplotlib.pyplot as plt 
import control as ct 
from control.matlab import *
import numpy as np 

df = pd.read_csv('data.csv')
df.columns 


m = 2 #kg 
J = .05 #kg m^2
tau = .2 #sec 
l = .15 #meters 
g = 9.81 #m/s^2
D = 0 # viscous damping , air drag 
s = tf('s');
C1 = tf(1, [tau, 1]);
C2 = l / (s*J);
C3 = 1/s
Ct = C1*C2
Ci = tf([3052, 3.256e04], [1, 2390]) #Kp = 13.6, Kd=1.27, Tf=.00418
FGi = Ci*C1*C2;
CLi  = feedback(FGi, 1)
FGo = CLi*C3;
Co = tf([7.845,  9.87], [1, 0]);
T = feedback(Co*FGo, 1);
t_eval = np.linspace(0, 5, 1000)
model, tout = step(T, t_eval)

fig, ax = plt.subplots() 
ax.plot(df['Time'], df['phi'], label = 'angle')
ax.plot(df['Time'], df['desired angle'], label='angle command')
ax.plot(tout+.58, model*max(df['desired angle']),
        linestyle='none', marker='*', markevery=10, label='Model step response')
ax.legend()
ax.set_xlabel('Time [sec]')
ax.set_ylabel('Angle [rad]')
ax.set_title('Angle Command and Response')




plt.show()

