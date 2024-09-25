#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Sep 25 13:26:52 2024

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
while True:
    # process key presses and mouse clicks
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            quit()
        elif event.type == pygame.MOUSEBUTTONDOWN:
            if (disturbance == 0):
                disturbance = 1
            else:
                disturbance = 0
        elif event.type == pygame.KEYDOWN:
            if pygame.K_0 <= event.key <= pygame.K_9:
                desired_speed = (event.key - pygame.K_0)/3

    # propagate the controller dynamics
    for k in range(0, Fc_FPS_ratio):
        e = desired_speed - angular_speed
        istate += T_c * e
        control_command = Kp*e + Ki*istate
        for j in range(0, Fphys_FPS_ratio//Fc_FPS_ratio):
            # propagate the plant dynamics
            angular_speed += (-10*angular_speed + 100*(control_command + disturbance))*T_phys
            angle += angular_speed*T_phys
# log the data

angular_speed_timeseries.append(angular_speed)
desired_speed_timeseries.append(desired_speed)
if (len(desired_speed_timeseries)>200):
    angular_speed_timeseries.pop(0)
    desired_speed_timeseries.pop(0)
    # write the text(s) and draw everything
    screen.fill((0, 0, 40))
    ## draw the objects onto the screen
    #draw the angles
    xpos=200*math.cos(angle)+window_width//2
    ypos =200*math.sin(angle)+window_height//2
    pygame.draw.circle(screen, (100, 100, 100), (window_width // 2, window_height // 2), 200, 1)
    pygame.draw.aaline(screen, (255, 100, 100), (window_width // 2, window_height // 2), (xpos, ypos))
    pygame.draw.aaline(screen, (255, 100, 100), (window_width // 2+1, window_height // 2+1), (xpos, ypos))
    pygame.draw.aaline(screen, (255, 100, 100), (window_width // 2-1, window_height // 2-1), (xpos, ypos))
    pygame.draw.aaline(screen, (255, 100, 100), (window_width // 2+2, window_height // 2+2), (xpos, ypos))
    pygame.draw.aaline(screen, (255, 100, 100), (window_width // 2-2, window_height // 2-2), (xpos, ypos))
    # draw the plot
    pygame.draw.aaline(screen, (255,255,255), (10, 270), (280, 270))# X-axis
    pygame.draw.aaline(screen, (255,255,255), (40, 180), (40, 380))# Y-axis
    scale = 30
    for i in range(1, len(desired_speed_timeseries)):
        pygame.draw.aaline(screen, (155,50,50),
                           ((i - 1) + 40, 270 - desired_speed_timeseries[i - 1] * scale),
                           (i + 40, 270 - desired_speed_timeseries[i] * scale))
        pygame.draw.aaline(screen, (255,255,255),
                           ((i - 1) + 40, 270 - angular_speed_timeseries[i - 1] * scale),
                           (i + 40, 270 - angular_speed_timeseries[i] * scale))

    pygame.display.flip()
    clock.tick(FPS)