#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Sep 25 13:24:34 2024

@author: erik
"""
# simple interactive motor speed control simulation

import pygame
import math

# step sizes. Everything scales on FPS

FPS =60# frames per second
Fc_FPS_ratio = 10# controller update rate vs. FPS
Fphys_FPS_ratio = 100# physics update rate vs. FPS
T_phys = 1 / (FPS*Fphys_FPS_ratio)
T_c = 1 / (FPS*Fc_FPS_ratio)
# try Kp=0, Ki=10  and Kp=0.1, Ki=1
Kp = 0
Ki = 10
# initialize states/outputs
desired_speed = 0# setpoint angle to be tracked, initially 0
angle = 0# initial condition for the angle
angular_speed = 0# initial condition for the angular speed
disturbance = 0# disturbance term, initially none present
angular_speed_timeseries = []
desired_speed_timeseries = []
# Initialize Pygame
pygame.init()
info = pygame.display.Info()
window_width = info.current_w-20
window_height = info.current_h-80
screen = pygame.display.set_mode((window_width, window_height))
clock = pygame.time.Clock()
istate = 0# integrator state
# Game loop
