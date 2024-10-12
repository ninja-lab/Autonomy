# The drone class
# Copyright 2024, Prof. Hamid Ossareh @ University of Vermont

import math

class Quadcopter:
    def __init__(self, Ts):
        # Physical Properties of the quadcopter.
        self.m = 2                # mass (kg)
        self.J = 0.05             # mass moment of inertia (kg m^2)
        self.motorTC = 0.2        # motor time constant (1/s)
        self.l = 0.15             # length of each arm (m)
        self.gravity = 9.81       # acceleration due to gravity (m/s^2)
        self.fricCoef = 0         # viscous friction of air on the body (damping coefficient)
        self.Ts = Ts

        # Initialize the states
        self.phi = 0              # roll angle
        self.phi_dot = 0          # roll rate
        self.x = 0                # initial horizontal position on the screen (0,0) is the center
        self.y = 0                # initial vertical position on the screen (0,0) is the center
        self.xdot = 0             # horizontal velocity
        self.ydot = 0             # vertical velocity
        self.motor1Thrust = self.m * self.gravity / 2            # initial motor 1 thrust
        self.motor2Thrust = self.motor1Thrust                    # initial motor 2 thrust

    # Update the dynamics
    def update(self, u1, u2):
        # Update motor dynamics and compute thrusts/moments
        self.motor1Thrust = self.motor1Thrust + self.Ts * (1 / self.motorTC) * (-self.motor1Thrust + u1)
        self.motor2Thrust = self.motor2Thrust + self.Ts * (1 / self.motorTC) * (-self.motor2Thrust + u2)

        # Update roll dynamics
        vehicleMoment = (self.motor1Thrust - self.motor2Thrust) * self.l 
        self.phi_dot = self.phi_dot + (1 / self.J) * self.Ts * (vehicleMoment)
        self.phi = self.phi + self.Ts * self.phi_dot

        # Calculate the force in the x and y directions (force applied to center of mass)
        yForce = (self.motor1Thrust + self.motor2Thrust) * math.cos(self.phi) - self.m * self.gravity
        xForce = (self.motor1Thrust + self.motor2Thrust) * math.sin(self.phi) 

        # Update position dynamics
        self.xdot = self.xdot + self.Ts * (1 / self.m) * (xForce - self.xdot * self.fricCoef)
        self.x = self.x + self.Ts * self.xdot  

        self.ydot = self.ydot + (1 / self.m) * self.Ts * (yForce - self.ydot * self.fricCoef)
        self.y = self.y + self.Ts * self.ydot

