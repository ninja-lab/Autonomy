#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Aug 31 10:07:59 2024

@author: erik
"""
import control as ct 
from control.matlab import *
import numpy as np 
import matplotlib.pyplot as plt 
C = ct.tf([9], [1]) 
G = ct.tf([1], [1, 1])
FG = ct.series(C,G )
#print(FG)
T = ct.feedback(FG)
#print(T)
t_eval = np.linspace(0, 2, 100)
yout, t_eval2 = step(T, t_eval )

fig, ax = plt.subplots()
ax.plot(t_eval2, yout)
ax.set_ylabel('Response Amplitude')
ax.set_xlabel('Time [sec]')
ax.set_ylim(0, 1)
ax.set_title(f'Step Response in Python')#' of {str(T)[str(T).find("\n\n\n"):]} ')
plt.show()
#part (e)
Kp = 19
Ki = 100
C = ct.tf([Kp, Ki], [1, 0])
G = ct.tf([1], [1, 1])
FG = ct.series(C,G )
print(C)
T = ct.feedback(FG)
print(T)
#the ZPK form provided gain isn't actually the gain, since the zeros terms 
#are in the numerator like s+z, and the denominator terms too
print(tf2zpk(T.num[0][0], T.den[0][0]))
