# Differential drive robot controller. Implements PID on two motors and trajectory tracking + AO. 
# Clips desired speeds to 2 m/s, which somewhat avoids motor saturation at steady-state
# Copyright 2024, Prof. Hamid Ossareh @ University of Vermont

from PID import PID
import numpy as np

class Controller:

    def __init__(self, Ts):
        # parameters
        self.robot_R = 0.1
        self.robot_L = 0.1
        self.max_speed = 2
        self.K_phi = 2
        self.K_e = 1
        self.zero_threshold = 1e-8
        self.Ts = Ts

        # motor controllers are PI with anti-windup
        self.leftWheelController = PID(Kp=1/3, Ki=10/3, Kd=0, Ts = Ts, umax = 10, umin = -10, Kaw = 1)
        self.rightWheelController = PID(Kp=1/3, Ki=10/3, Kd=0, Ts = Ts, umax = 10, umin = -10, Kaw = 1)

    # Update function. Goal: go to goal
    # measurements: left and right motor speeds, robot heading and x/y positions
    # outputs: left and right motor voltages
    def update(self, omega_l, omega_r, phi, x, y, x_des, y_des, ux_des, uy_des, points):
     
        def norm(x, y):
            return (x**2+y**2)**0.5

        # returns the gain adjusted on U_AO
        def Kp_obstacle(x):
            return np.clip((-x + self.max_speed), 0, self.max_speed) / max(0.0001, x)

        # assign velocity vector
        U_AO = np.zeros(2)
        for j in range(len(points)):
            p = points[j]
            position_error = np.array([p[0] - x, p[1] - y])
            velocity_vector_temp = -Kp_obstacle(np.linalg.norm(position_error)) * position_error
            U_AO += velocity_vector_temp

        # compute GTG 
        # U_GTG = self.K_e*np.array([x_des - x, y_des - y])
        U_GTG = self.K_e*np.array([x_des - x, y_des - y]) + np.array([ux_des, uy_des])

        # clip speeds to 2m/s
        def clip_speed(u):
            norm_tmp = np.linalg.norm(u)
            if norm_tmp > self.max_speed:
                u = u / norm_tmp * self.max_speed
            return u

        U_GTG = clip_speed(U_GTG)
        U_AO  = clip_speed(U_AO)

        # Blend GTG and AO
        sigma = min(1, np.linalg.norm(U_AO)/self.max_speed)
        u = sigma * U_AO + (1 - sigma) * U_GTG
        ux, uy = u[0], u[1]
        s_des = norm(ux, uy)
        if (s_des < self.zero_threshold):
            s_des = 0
            phi_des = phi
        else:
            phi_des = np.arctan2(uy, ux)

        # proportional control for heading tracking. 
        error = phi_des - phi
        error = np.atan2(np.sin(error), np.cos(error))    # Wrap angles! 
        omega_des = error*self.K_phi

        # motor setpoints
        omega_r_des = (s_des + omega_des*self.robot_L)/self.robot_R
        omega_l_des = (s_des - omega_des*self.robot_L)/self.robot_R
        
        # update motor PIDs
        V_r = self.rightWheelController.update(omega_r_des, omega_r)
        V_l = self.leftWheelController.update(omega_l_des, omega_l)

        return V_r, V_l