# Planar drone simulation
# Copyright 2024, Prof. Hamid Ossareh @ University of Vermont

import pygame
import pygame_widgets
import csv
import math
from utilities import draw, remapMouse
from pygame_widgets.button import Button
#from quadcopter import Quadcopter
from quadcopter import Quadcopter
from controller import Controller
import pdb

# Initialize Pygame
pygame.init()

# Screen settings
info = pygame.display.Info()
window_width = info.current_w-20
window_height = info.current_h-110
screen = pygame.display.set_mode((window_width, window_height))
pygame.display.set_caption('Quadcopter Simulator')
font = pygame.font.SysFont("Arial", 18)

# logging variables
data = []
m1_log = []
m2_log = []

button = Button(
    screen, window_width-250, 30, 220, 80, text='Click to save to CSV',
    fontSize=24, margin=20, 
    inactiveColour=(255, 255, 255),
    pressedColour=(155, 255, 155), radius=20,
    onClick=lambda: save_data_log(data)
)

# Log button functionality
def save_data_log(data):
    with open('data.csv', 'w', newline='') as file:
        writer = csv.writer(file)
        fieldnames = ["Time", "x pos", "y pos", "phi", "motor1 thrust", "motor2 thrust", 
                       "desired angle", "desired thrust"]
        writer.writerow(fieldnames)
        writer.writerows(data)

# Set the frame rate
FPS = 60
clock = pygame.time.Clock()

# Variables
FsMultiplier = 5
Ts = 1 / (60 * FsMultiplier)
running = False
simTime = 0

# create quadcopter and controller objects
quad1 = Quadcopter(Ts)
controller = Controller(Ts)

# Main loop
while True:
    # set the angle and altitude setpoints based on mouse location
    desired_angle = remapMouse(pygame.mouse.get_pos()[0] - window_width / 2, window_width)/600
    thrust_input = -remapMouse(pygame.mouse.get_pos()[1] - window_height / 2, window_width)/50
    desired_thrust = quad1.m*quad1.gravity + thrust_input
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            quit()
        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_SPACE:
                running = not running

    # if simulation is running (spacebar has been pressed)
    if running:
        if (simTime > 100):
            running = not running

        for k in range(FsMultiplier):
            # log the data
            row_data = [simTime, quad1.x, quad1.y, quad1.phi, 
                        quad1.motor1Thrust, quad1.motor2Thrust,
                        desired_angle, desired_thrust]
            data.append(row_data)

            # update the plant and controller dynamics
            m1, m2 = controller.update(quad1.phi, quad1.phi_dot, desired_angle, desired_thrust)
            #breakpoint()
            quad1.update(m1, m2)
            #breakpoint()
            simTime += Ts
        
        # log the data for plotting
        m1_log.append(m1)
        m2_log.append(m2)
        if (len(m1_log)>200):
            m1_log.pop(0)
            m2_log.pop(0)

    # draw everything and refresh the page
    
    draw(screen,font, quad1, desired_angle, thrust_input, running, simTime, clock, m1_log, m2_log)
    pygame_widgets.update(event)
    pygame.display.flip()
    clock.tick(FPS)
