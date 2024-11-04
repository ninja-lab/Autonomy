#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Nov  2 10:10:04 2024

@author: erik
"""
import pandas as pd 
import matplotlib.pyplot as plt 

import numpy as np 

df1 = pd.read_csv('data_linear.csv') #Ke = 5
df2 = pd.read_csv('data_spline.csv') #Ke = 2
df3 = pd.read_csv('data_spline2.csv') 
fig, ax = plt.subplots()
ax.plot(df1['Time'], df1['Left Voltage'], label = 'Linear')
ax.plot(df2['Time'], df2['Left Voltage'], label='Spline')
ax.plot(df3['Time'], df3['Left Voltage'], label='Spline2')
ax.set_xlabel('Time [sec]')
ax.set_title('Left Motor Command')
ax.set_ylabel('Motor Input Voltage [V]')
ax.legend()

fig, ax = plt.subplots()
ax.plot(df1['Time'], df1['Right Voltage'], label = 'Linear')
ax.plot(df2['Time'], df2['Right Voltage'], label='Spline')
ax.plot(df3['Time'], df3['Right Voltage'], label='Spline2')
ax.set_xlabel('Time [sec]')
ax.set_title('Right Motor Command')
ax.set_ylabel('Motor Input Voltage [V]')
ax.legend()



plt.show()