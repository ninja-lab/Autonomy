# Planar drone simulation
# Copyright 2024, Prof. Hamid Ossareh @ University of Vermont

import pygame
import pygame_widgets
import csv
from utilities import draw
from pygame_widgets.button import Button
import numpy as np
from matplotlib.patches import Polygon
from shapely.geometry import Polygon as ShapelyPolygon, Point

from quadcopter import Quadcopter
from controller import Controller

####################################################
# Set the frame rate
FPS = 60
clock = pygame.time.Clock()

# Set the stepsize
FsMultiplier = 5
Ts = 1 / (60 * FsMultiplier)

# create quadcopter and controller objects
quad1 = Quadcopter(Ts)
controller = Controller(Ts)
sensor_radius = 3                   # sensing radius for obstacle detection
####################################################

# Initialize Pygame
pygame.init()

# Screen settings
info = pygame.display.Info()
window_width = info.current_w-20
window_height = info.current_h-110
screen = pygame.display.set_mode((window_width, window_height))
pygame.display.set_caption('Quadcopter Simulator')
font = pygame.font.SysFont("Arial", 18)

# Obstacles
scale = 0.3
pgons = [
    Polygon([[-21*scale, -21*scale], [21*scale, -21*scale], [21*scale, -22*scale], [-21*scale, -22*scale]]),
    Polygon([[-21*scale, 21*scale], [21*scale, 21*scale], [21*scale, 22*scale], [-21*scale, 22*scale]]),
    Polygon([[-21*scale, 21*scale], [-21*scale, -21*scale], [-22*scale, -21*scale], [-22*scale, 21*scale]]),
    Polygon([[21*scale, 21*scale], [21*scale, -21*scale], [22*scale, -21*scale], [22*scale, 21*scale]]),
    Polygon([[5*scale, 5*scale], [10*scale, 5*scale], [10*scale, 10*scale], [5*scale, 10*scale,]]),
    Polygon([[12*scale, 2*scale], [15*scale, 2*scale], [15*scale, -10*scale], [12*scale, -10*scale]]),
    Polygon([[-10*scale, -15*scale], [-7*scale, -15*scale], [-7*scale, 10*scale], [-10*scale, 10*scale]])
]

# Helper function to compute distances from obstacles
def distance_to_polygon(pgon, x, y):
    point = Point(x, y)
    shapely_pgon = ShapelyPolygon(pgon.get_xy())
    distance = point.distance(shapely_pgon)
    nearest_point = shapely_pgon.exterior.interpolate(shapely_pgon.exterior.project(point))
    return distance, nearest_point.x, nearest_point.y

# Function to find distance to nearby objects
def detect_obstacles(x, y):
    closest_points = []
    for j in range(len(pgons)):
        dist, xtmp, ytmp = distance_to_polygon(pgons[j], x, y)
        if (dist==0):
            print('\033[31mCrash detected!\033[0m')
        elif (dist < sensor_radius):
            closest_points.append([xtmp, ytmp])
    return closest_points

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
        fieldnames = ["Time", "x", "y", "xdot", "ydot", "phi", "motor1 thrust", "motor2 thrust", 
                       "desired x", "desired y"]
        writer.writerow(fieldnames)
        writer.writerows(data)

running = False
simTime = 0
target = np.array([0.0, 0.0])       # Target (goal) point, initially the origin
scaled_target = np.array([0, 0])

# Main loop
while True:

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            quit()
        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_SPACE:
                running = not running
        elif event.type == pygame.MOUSEBUTTONDOWN:
            mx, my = pygame.mouse.get_pos()
            target = np.array([mx-window_width//2, my-window_height//2])
            scaled_target = target.copy()/50
            scaled_target[1] = -scaled_target[1]

    # if simulation is running (spacebar has been pressed)
    if running:
        if (simTime > 500):
            running = not running

        for k in range(FsMultiplier):
            # update the plant and controller dynamics
            closest_points = detect_obstacles(quad1.x, quad1.y)                
            m1, m2 = controller.update(quad1.phi, quad1.phi_dot, quad1.x, quad1.y,
                                       quad1.xdot, quad1.ydot, scaled_target[0], scaled_target[1], closest_points)
            quad1.update(m1, m2)
            simTime += Ts

        # log the data
        row_data = [simTime, quad1.x, quad1.y, quad1.xdot, quad1.ydot, quad1.phi, 
                    quad1.motor1Thrust, quad1.motor2Thrust,
                    scaled_target[0], scaled_target[1]]
        data.append(row_data)
        
        # log the data for plotting
        m1_log.append(m1)
        m2_log.append(m2)
        if (len(m1_log)>200):
            m1_log.pop(0)
            m2_log.pop(0)

    # draw everything and refresh the page
    draw(screen, font, quad1, running, simTime, clock, m1_log, m2_log, target, pgons)
    pygame_widgets.update(event)
    pygame.display.flip()
    clock.tick(FPS)
