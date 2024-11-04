# Differential drive robot simulation, intended for EE5550.
# Copyright 2024, Prof. Hamid Ossareh @ University of Vermont
# Robot dynamics, graphics, and path following with pure pursuit+AO+motor controllers.

import numpy as np
from matplotlib.patches import Polygon
from shapely.geometry import Polygon as ShapelyPolygon, Point
from robot import Robot
from controller import Controller
import draw
import pygame

# frame rate, and physics/control sampling periods
FPS = 60
physics_updates_per_controller_update = 5  #about 3ms
physics_updates_per_frame_update = 10      #about 1.5ms
T_phys = 1/FPS/physics_updates_per_frame_update
T_cont = 1/FPS/(physics_updates_per_frame_update/physics_updates_per_controller_update)

####################################################################################

robot = Robot(T_phys, 0, 3, 0)      # robot object
controller = Controller(T_cont)     # controller object
V_r, V_l = 0, 0                     # initialize initial voltages
sensor_radius = 3                   # sensing radius for obstacle detection

# Obstacles
pgons = [
    Polygon([[-21, -21], [21, -21], [21, -22], [-21, -22]]),
    Polygon([[-21, 21], [21, 21], [21, 22], [-21, 22]]),
    Polygon([[-21, 21], [-21, -21], [-22, -21], [-22, 21]]),
    Polygon([[21, 21], [21, -21], [22, -21], [22, 21]]),
    Polygon([[12, 2], [15, 2], [15, -10], [12, -10]]),
    Polygon([[-10, -15], [-7, -15], [-7, 10], [-10, 10]])
]

# Helper function to compute distances from obstacles
def distance_to_polygon(pgon, x, y):
    point = Point(x, y)
    shapely_pgon = ShapelyPolygon(pgon.get_xy())
    distance = point.distance(shapely_pgon)
    nearest_point = shapely_pgon.exterior.interpolate(shapely_pgon.exterior.project(point))
    return distance, nearest_point.x, nearest_point.y

def detect_obstacles(x, y):
    closest_points = []
    for j in range(len(pgons)):
        dist, xtmp, ytmp = distance_to_polygon(pgons[j], x, y)
        if (dist==0):
            print('\033[31mCrash detected!\033[0m')
        elif (dist < sensor_radius):
            closest_points.append([xtmp, ytmp])
    return closest_points

# For logging and plotting
data = {
    'time': [0], 'x': [robot.x], 'y': [robot.y], 'target': [controller.target],
    'heading': [robot.heading], 'speed': [0], 'left voltage': [0], 'right voltage': [0]
}

################################ 
# new for this homework
path = np.array([[2, 2], [5, 8], [10, 12], [12, 12], [14, 12], [17, 10], [16,7], [0, 0]])
path = path.T

# Find the path for plotting
tr = []
for t in np.arange(0, len(path.T), 0.01):
    x, y = np.interp(t, range(0,len(path.T)), path[0,:]), np.interp(t, range(0,len(path.T)), path[1,:])
    if not ((x,y) in tr):
        tr.append((x,y))
################################

sim_time = 0        # Elapsed time since simulation started running (after pressing spacebar)
running = False     # Spacebar has not yet been pressed
frame_skip = 1      # how many frames to skip while plotting (to make the sim run faster)
# Main loop
while True: 
    # first process mouse and keyboard events. 
    events = pygame.event.get()
    for event in events:
        if event.type == pygame.QUIT:
            pygame.quit()
            quit()
        elif event.type == pygame.KEYDOWN:
            # spacebar runs/stops the sim
            if event.key == pygame.K_SPACE:
                running = not running
            # number keys determine how fast the sim runs
            elif pygame.K_0 <= event.key <= pygame.K_9:
                frame_skip = event.key - pygame.K_0
    
    if running:
        if sim_time > 500:           # run/log up to 500 seconds
            running = not running

        # run the physics and controller at the correct sampling period
        for j in range(0, physics_updates_per_frame_update*frame_skip):
            if (j % physics_updates_per_controller_update == 0):
                # First check distance to all obstacles and store closest approach
                # If there is collision, print to terminal!
                closest_points = detect_obstacles(robot.x, robot.y)    
                V_r, V_l = controller.update(robot.omega_l, robot.omega_r, robot.heading, robot.x, robot.y, path, closest_points)

            robot.update(V_r, V_l)
            sim_time += T_phys

        new_data = {
            'time': sim_time, 'x':  robot.x, 'y': robot.y, 'target': controller.target, 
            'heading': robot.heading, 'speed': robot.speed, 'left voltage': V_l, 'right voltage': V_r
        }
        for k, v in new_data.items():
            data[k].append(v)
        
    # draw everything to the screen
    draw.draw(data, pgons, controller.target, FPS, running, events, tr)


