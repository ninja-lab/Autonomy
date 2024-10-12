# The drone class
# Copyright 2024, Prof. Hamid Ossareh @ University of Vermont

import math
sin = math.sin
cos = math.cos 
pi = math.pi 
class Quadcopter:
    def __init__(self, Ts):
        # Physical Properties of the quadcopter.
        self.m = 2                # mass (kg)
        self.J = .05             # mass moment of inertia (kg m^2)
        self.motorTC = 0.2        # motor time constant (1/s)
        self.l = 0.15             # length of each arm (m)
        self.gravity = 9.81       # acceleration due to gravity (m/s^2)
        self.fricCoef = 0         # viscous friction of air on the body (damping coefficient)
        self.Ts = Ts
        self.D = self.fricCoef
        # Initialize the states
        self.motor1Thrust = self.m * self.gravity / 2            # initial motor 1 thrust
        self.motor2Thrust = self.motor1Thrust                    # initial motor 2 thrust
        # EE5550: add your own states 
        self.Fdif = 0 
        self.phi = 0 # robot phi (angle)
        self.x = 0              # x position of the robot
        self.y = 0              # y position of the robot
        self.z1 = 0             #the same as xdot 
        self.z2 = 0             #the same as ydot 
        self.z1dot = 0                  # x acceleration 
        self.z2dot = 0                  # y acceleration 
        self.phi_dot = 0                  # angular speed of the robot body

    # Update the dynamics. v1 and v2 are the two motor commands
    def update(self, v1, v2):
        # EE5550: Update motor dynamics and compute thrusts/moments
        self.motor1Thrust = self.motor1Thrust*(1 - self.Ts/self.motorTC) + \
            self.Ts*v1/self.motorTC
        self.motor2Thrust = self.motor2Thrust*(1 - self.Ts/self.motorTC) + \
            self.Ts*v2/self.motorTC
        self.Fdif = self.motor1Thrust - self.motor2Thrust
        self.Fcol = self.motor1Thrust + self.motor2Thrust 
        # EE5550: Update roll dynamics
        self.phi_dot = self.phi_dot + self.Ts*self.Fdif*self.l/self.J
        # EE5550: Calculate the force in the x and y directions (force applied to center of mass)
        self.phi = self.phi + self.Ts*self.phi_dot
        #acceleration in x direction:
        self.z1dot = -(self.D/self.m)*self.z1 + self.Fcol*sin(self.phi)/self.m
        self.z1 = self.z1 + self.Ts*self.z1dot 
        #acceleration in y direction:
        self.z2dot = -(self.D/self.m)*self.z2 + (self.Fcol*cos(self.phi) - \
            self.m*self.gravity)/self.m 
        self.z2 = self.z2 + self.Ts*self.z2dot 
        # EE5550: Update position dynamics
        self.x = self.x + self.Ts*self.z1
        self.y = self.y + self.Ts*self.z2
    