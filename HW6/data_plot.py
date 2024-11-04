#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Nov  2 17:59:04 2024

@author: erik
"""
import pandas as pd 
import matplotlib.pyplot as plt 

import numpy as np 

df = pd.read_csv('data.csv') #Ke = 5


fig, ax = plt.subplots()
ax.plot(df['Time'], df['Speed'])
#ax.plot(df2['Time'], df2['Left Voltage'], label='Spline')
ax.set_xlabel('Time [sec]')
ax.set_title('Robot Speed')
ax.set_ylabel('Speed [m/s]')
ax.legend()

# fig, ax = plt.subplots()
# ax.plot(df1['Time'], df1['Right Voltage'], label = 'Linear')
# ax.plot(df2['Time'], df2['Right Voltage'], label='Spline')
# ax.set_xlabel('Time [sec]')
# ax.set_title('Right Motor Command')
# ax.set_ylabel('Motor Input Voltage [V]')
# ax.legend()



plt.show()