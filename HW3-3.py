#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Sep 28 11:05:19 2024

@author: erik
"""


# simple interactive motor speed control simulation
import pygame
import math

# step sizes. Everything scales on FPS
pi = math.pi
FPS =60# frames per second
Fc_FPS_ratio = 10# controller update rate vs. FPS
Fphys_FPS_ratio = 100# physics update rate vs. FPS
T_phys = 1 / (FPS*Fphys_FPS_ratio)
Tc = 1 / (FPS*Fc_FPS_ratio)

# Kp = 13.5
# Ki = 54.00971
# Kd = .849
# Tf = .0001
Kp = 4.22
Ki = 13.77
Kd = .315
Tf = .002941
b = .1
J = .01
k = 1

class PIDF: 
    def __init__ (self, Kp=1, Ki=1, Kd=1, Tf=.1, Tc=0.01, Kt=1,
                  sat_min= -math.inf, sat_max = math.inf, initial_condition=0): 
        self.Kp = Kp 
        self.Ki = Ki 
        self.Tc = Tc 
        self.Kd = Kd 
        self.Tf = Tf
        self.sat_max = sat_max
        self.sat_min = sat_min 
        self.Kt = Kt
        self.integrator_state = initial_condition 
        self.e_previous = 0
        self.ud = 0
        self.es_previous = 0 
        
    def update(self, r, y): 
        error = r - y 
  
        self.ud = (1/self.Tf)*(error - self.e_previous-(self.Tc-self.Tf)*self.ud)
        
        self.integrator_state = self.integrator_state + \
            self.Tc*(self.Ki*error + self.Kt*self.es_previous)
        
        self.ctl_cmd = self.Kp*error + self.Ki*self.integrator_state + self.Kd*self.ud
        self.ctl_cmd_sat = max( min (self.ctl_cmd, self.sat_max), self.sat_min)
        
        self.es = self.ctl_cmd_sat - self.ctl_cmd

        self.es_previous = self.es
        self.e_previous = error
        return self.ctl_cmd_sat


controller = PIDF(Kp=Kp, Ki=Ki, Kd=Kd, Tf=Tf, Tc=Tc, Kt=10, sat_min=-100, 
                  sat_max=100)

# initialize states/outputs
#desired_speed = 0# setpoint angle to be tracked, initially 0
desired_angle= 0
angle = 0# initial condition for the angle
angular_speed = 0# initial condition for the angular speed

disturbance = 0# disturbance term, initially none present
angular_speed_timeseries = []
desired_speed_timeseries = []
angle_timeseries = []
desired_angle_timeseries = []
# Initialize Pygame
pygame.init()
info = pygame.display.Info()
window_width = info.current_w-600
window_height = info.current_h-800
screen = pygame.display.set_mode((window_width, window_height))
clock = pygame.time.Clock()

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
                desired_angle = (event.key - pygame.K_0)*2*pi/(pygame.K_9- pygame.K_0)
                print(f'desired angle: {desired_angle}')

    # propagate the controller dynamics
    for k in range(0, Fc_FPS_ratio):

        control_command = controller.update(desired_angle, angle)

        for j in range(0, Fphys_FPS_ratio//Fc_FPS_ratio):
            # propagate the plant dynamics
            angular_speed += (-(b/J)*angular_speed + (k/J)*(control_command + disturbance))*T_phys
            angle += angular_speed*T_phys
# log the data
    angle_timeseries.append(angle)
    desired_angle_timeseries.append(desired_angle)
    if (len(desired_angle_timeseries)>200):
        angle_timeseries.pop(0)
        desired_angle_timeseries.pop(0)
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
        for i in range(1, len(desired_angle_timeseries)):
            pygame.draw.aaline(screen, (155,50,50),
                               ((i - 1) + 40, 270 - desired_angle_timeseries[i - 1] * scale),
                               (i + 40, 270 - desired_angle_timeseries[i] * scale))
            pygame.draw.aaline(screen, (255,255,255),
                               ((i - 1) + 40, 270 - angle_timeseries[i - 1] * scale),
                               (i + 40, 270 - angle_timeseries[i] * scale))
    
        pygame.display.flip()
        clock.tick(FPS)