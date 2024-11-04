# The drone controller

from PID import PID
import numpy as np


class Controller:
    def __init__(self, Ts):
        # Nominal parameters that we can use for the quadcopter.
        self.Ts = Ts
        self.m = 2          # mass in kg
        self.J = 0.05       # mass moment of inertia
        self.motorTC = 0.2  # motor time constant
        self.l = 0.15       # length of each arm
        self.gravity = 9.81 # acceleration due to gravity
        self.max_speed = 2
        self.zero_threshold = 1e-8
        self.K_phi = .5
        self.K_e = 1 #1
        self.K_thr = 10
        # EE 5550: Initialize the inner and outer loop angle controllers here
        self.angleRateController = PID(Kp=20, Ki=0, Kd=4, N=0.02, Ts=self.Ts, umax=1000,
                                umin = -1000, Kt=0)
        self.angleController = PID(Kp=10, Ki=10, Kd=0, N=0.2, Ts=self.Ts, umax=1000,
                                umin=-1000, Kt=0)

        #self.angleRateController = PID(Kp = 13.6, Ki = 0, Kd=1.27,
        #                               Tf = .00418, Tc=Ts, Kt=0)
        #self.angleController =PID(Kp=7.8455, Ki = 9.8703, Kd=0, Tc=Ts,Kt=0)

    def update(self, phi, phi_dot, x, y, x_dot, y_dot, x_des, y_des, points):
        def norm(x, y):
            return (x**2+y**2)**0.5
        # U_AO velocity vector
        # returns the gain adjusted on U_AO
        def Kp_obstacle(x):
            return np.clip((-x + self.max_speed), 0, self.max_speed) / max(0.0001, x)
        def Kp_obstacle_2(x):
            return np.clip((-x + self.max_speed), 0, self.max_speed) / max(.1, x)
        def Kp_obstacle_3(x):
            return np.clip((-x + self.max_speed), 0, self.max_speed) 
        U_AO = np.zeros(2)
        for j in range(len(points)):
            p = points[j]
            position_error = np.array([p[0] - x, p[1] - y])
            velocity_vector_temp = -Kp_obstacle_3(np.linalg.norm(position_error)) * position_error
            U_AO += velocity_vector_temp        
        # U_GTG velocity vector
        # compute GTG 
        U_GTG = self.K_e*np.array([x_des - x, y_des - y])
        # assign velocity vector
        # clip U_AO and U_GTG to 2m/s
        # clip speeds to 2m/s
        def clip_speed(u):
            norm_tmp = np.linalg.norm(u)
            if norm_tmp > self.max_speed:
                u = u / norm_tmp * self.max_speed
            return u

        U_GTG = clip_speed(U_GTG)
        U_AO  = clip_speed(U_AO)
        # blend U_AO and U_GTG to compute ux and uy
        sigma = min(1, np.linalg.norm(U_AO)/self.max_speed)
        u = sigma * U_AO + (1 - sigma) * U_GTG
        ux, uy = u[0], u[1]

        '''
        write the code to convert ux and uy into desired_angle (i.e., ϕdes ) and
        desired_thrust (i.e., ucol ). Use proportional gain 0.5 for ϕdes and 10
        for ucol .
        '''
        x_dot_des = ux #name convention change
        x_dot_error = x_dot_des - x_dot #from block diagram
        desired_angle = self.K_phi*x_dot_error  #from block diagram
        
        y_dot_des = uy 
        y_dot_error= y_dot_des - y_dot #from block diagram
        desired_thrust = self.K_thr*y_dot_error + self.m*self.gravity/np.cos(phi) #from block diagram
        
        # update the angle rate and angle controllers to compute the desired differential thrust
        rin = self.angleController.update( desired_angle, phi )         
        udif = self.angleRateController.update(rin, phi_dot)
        # compute actuator commands

        motor1_desired_thrust = .5*(desired_thrust + udif) #by definition
        motor2_desired_thrust = .5*(desired_thrust - udif) #by definition
        return motor1_desired_thrust, motor2_desired_thrust

