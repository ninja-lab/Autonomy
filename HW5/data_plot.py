#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Oct 12 12:36:33 2024

@author: erik
"""
import pandas as pd 
import matplotlib.pyplot as plt 
import control as ct 
from control.matlab import *
import numpy as np 

df = pd.read_csv('data.csv')
df.columns 

fig, ax = plt.subplots() 
ax.plot(df['Time'], df['xdot'], label = 'xdot')
ax.plot(df['Time'], df['ydot'], label='ydot')

ax.legend()
ax.set_xlabel('Time [sec]')
ax.set_ylabel('Velocity [m/s]')
ax.set_title('x and y quadcopter velocity tracking')
plt.show()