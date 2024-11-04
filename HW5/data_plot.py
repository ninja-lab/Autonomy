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

df = pd.read_csv('data_base1.csv') #Ke = 5
dfKe2 = pd.read_csv('dataKe2.csv') #Ke = 2
dfKe5 = pd.read_csv('dataKe5.csv') 
dfKe10 = pd.read_csv('dataKe10.csv') 
dfKphi2 = pd.read_csv('dataKphi2.csv') 
dfKpobs2 = pd.read_csv('dataKpobs2.csv')
dfKpobs3 = pd.read_csv('dataKpobs3.csv')
df.columns 

fig, ax = plt.subplots()
ax.plot(df['Time'], df['x'], label = 'x')
ax.plot(df['Time'], df['y'], label='y')
ax.set_xlabel('Time [sec]')
ax.set_title('Position Baseline')
ax.set_ylabel('x, y Position [m]')
ax.legend()

fig, ax = plt.subplots()
ax.plot(df['Time'], df['x'], label = 'baseline (large Kp_obs) x')
ax.plot(df['Time'], df['y'], label='baseline (large Kp_obs) y')
ax.plot(dfKpobs3['Time'], dfKpobs3['x'], label = 'small Kp_obs x')
ax.plot(dfKpobs3['Time'], dfKpobs3['y'], label='small Kp_obs y')
ax.plot(dfKpobs2['Time'], dfKpobs2['x'], label = 'medium Kp_obs x')
ax.plot(dfKpobs2['Time'], dfKpobs2['y'], label='medium Kp_obs y')

ax.set_xlabel('Time [sec]')
ax.set_title('Position with varying Kp_obstacle')
ax.set_ylabel('x, y Position [m]')
ax.legend()

fig, (ax1, ax3) = plt.subplots(2,1, sharex=True) 
ax2 = ax1.twinx() 
ax4 = ax3.twinx()
#ax1 for xdot, ax2 for x
#ax3 for ydot, ax4 for y 
ax1.plot(df['Time'], df['xdot'], 'r', label = r'$\dot{x}$')
ax3.plot(df['Time'], df['ydot'], 'r', label=r'$\dot{y}$')
ax2.plot(df['Time'], df['x'],'r--', label = 'x')
ax4.plot(df['Time'], df['y'], 'r--', label='y')

ax1.plot(dfKe2['Time'], dfKe2['xdot'], 'b')#, label = r'$\dot{x}$')
ax3.plot(dfKe2['Time'], dfKe2['ydot'], 'b')#, label=r'$\dot{y}$')
ax2.plot(dfKe2['Time'], dfKe2['x'],'b--')#, label = 'x')
ax4.plot(dfKe2['Time'], dfKe2['y'], 'b--')#, label='y')

ax1.plot(dfKe5['Time'], dfKe5['xdot'], 'g')#, label = r'$\dot{x}$')
ax3.plot(dfKe5['Time'], dfKe5['ydot'], 'g')#, label=r'$\dot{y}$')
ax2.plot(dfKe5['Time'], dfKe5['x'],'g--')#, label = 'x')
ax4.plot(dfKe5['Time'], dfKe5['y'], 'g--')#, label='y')

ax1.plot(dfKe10['Time'], dfKe10['xdot'], 'k')#, label = r'$\dot{x}$')
ax3.plot(dfKe10['Time'], dfKe10['ydot'], 'k')#, label=r'$\dot{y}$')
ax2.plot(dfKe10['Time'], dfKe10['x'],'k--')#, label = 'x')
ax4.plot(dfKe10['Time'], dfKe10['y'], 'k--')#, label='y')

# Adding colored annotations for Kp values in figure coordinates
ax1.text(0.15, 0.5, 'Ke = 1', color='red', fontsize=12, transform=ax1.transAxes)
ax1.text(0.15, 0.4, 'Ke = 2', color='blue', fontsize=12, transform=ax1.transAxes)
ax1.text(0.15, 0.3, 'Ke = 5', color='green', fontsize=12, transform=ax1.transAxes)
ax1.text(0.15, 0.2, 'Ke = 10', color='black', fontsize=12, transform=ax1.transAxes)
ax1.text(0.65, 0.5, 'dashes = position', color='black', fontsize=12, transform=ax1.transAxes)
ax1.text(0.65, 0.4, 'solid = velocity', color='black', fontsize=12, transform=ax1.transAxes)
ax3.set_xlabel('Time [sec]')
ax3.set_ylabel('y velocity [m/s]')
ax2.set_ylabel('x Position [m]')
ax1.set_ylabel('x velocity [m/s]')
ax4.set_ylabel('y position [m]')
ax1.set_title('quadcopter position and velocity towards upper left')
#ax.legend(loc=(.15,.1))
#ax2.legend(loc=(.35,.1))



fig, (ax1, ax2) = plt.subplots(2,1, sharex=True) 

Fcolbase = (df['motor1 thrust'] + df['motor2 thrust'])*.5
Fdiffbase = (df['motor1 thrust'] - df['motor2 thrust'])*.5
FcolKphi2 = (dfKphi2['motor1 thrust'] + dfKphi2['motor2 thrust'])*.5
FdiffKphi2 = (dfKphi2['motor1 thrust'] - dfKphi2['motor2 thrust'])*.5
ax1.plot(df['Time'], Fcolbase, 'r--', label = 'Collective Thrust')
ax1.plot(df['Time'], Fdiffbase, 'r-', label='Differential')
ax1.plot(dfKphi2['Time'], FcolKphi2, 'b--')
ax1.plot(dfKphi2['Time'], FdiffKphi2, 'b-')
#ax2=ax.twinx()
ax2.plot(df['Time'], df['phi'], 'r--', label = r'$K\phi$ = .5')
ax2.plot(dfKphi2['Time'], dfKphi2['phi'], 'b--', label = r'$K\phi$ = 2')
#ax1.legend()
ax2.legend(loc='lower right')
ax1.set_title('Thrusts and Angle $K_\phi$ Experimentation')
ax2.set_xlabel('Time [sec]')
ax1.set_ylabel('Motor Thrust [N]')
ax2.set_ylabel('Angle [rad]')
ax1.text(0.15, 0.2, r'$K\phi = .5$', color='red', fontsize=12, transform=ax1.transAxes)
ax1.text(0.15, 0.1, r'$K\phi = 2$', color='blue', fontsize=12, transform=ax1.transAxes)
ax1.text(.5, .1, '- - - Collective Thrust (dashed lines)', transform=ax1.transAxes )
ax1.text(.5, .2, '----- Differential Thrust (solid lines)', transform=ax1.transAxes )
ax1.set_xlim(0,3)
ax2.set_xlim(0,3)


fig, ax = plt.subplots()
ax.plot(df['Time'], df['xdot'], 'r', label = r'$\dot{x}$')
ax2=ax.twinx()
ax2.plot(df['Time'], df['phi'], 'k--', label = r'$\phi$')
ax.legend(loc=(.2,.1))
ax2.legend(loc='lower right')
ax.set_title(r'$\dot{x}$ and Angle $K_\phi$ Baseline')
ax.set_xlabel('Time [sec]')
ax.set_ylabel('x velocity [m/s]')
ax2.set_ylabel('Angle [rad]')

fig, ax = plt.subplots()
ax.plot(df['Time'], df['ydot'], 'r', label = r'$\dot{y}$')
ax2=ax.twinx()
ax2.plot(df['Time'], Fcolbase, 'k--', label = r'$F_{col}$')
ax.set_title(r'$\dot{y}$ and Collective Thrust $F_{col}$ Baseline')
ax.set_xlabel('Time [sec]')
ax.set_ylabel('y velocity [m/s]')
ax2.set_ylabel('Thrust [N]')
ax.legend(loc=(.5,.6))
ax2.legend(loc=(.5, .7))


max_speed = 2
x = np.linspace(0, 3)
def Kp_obstacle_base(x):
    return np.clip((-x + max_speed), 0, max_speed) / max(0.0001, x)
def Kp_obstacle_2(x):
    return np.clip((-x + max_speed), 0, max_speed) / max(.1, x)
def Kp_obstacle_3(x):
    return np.clip((-x + max_speed), 0, max_speed) 
fig, ax = plt.subplots()
ax.plot(x, np.vectorize(Kp_obstacle_base)(x), label='baseline')
ax.plot(x, np.vectorize(Kp_obstacle_2)(x), label='mod1')
ax.plot(x, np.vectorize(Kp_obstacle_3)(x), label='mod2')
ax.set_title('Kp_obstacle(x)')
ax.set_xlabel('Distance to Obstacle [m]')
ax.set_ylabel('Kp')
ax.set_ylim(0,50)
ax.legend()
plt.show()