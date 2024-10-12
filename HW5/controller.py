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

        # EE 5550: Initialize the inner and outer loop angle controllers here
        self.angleRateController = PID(Kp=20, Ki=0, Kd=4, N=0.02, Ts=self.Ts, umax=1000,
                                umin = -1000, Kt=0)
        self.angleController = PID(Kp=10, Ki=10, Kd=0, N=0.2, Ts=self.Ts, umax=1000,
                                umin=-1000, Kt=0)

        #self.angleRateController = PID(Kp = 13.6, Ki = 0, Kd=1.27,
        #                               Tf = .00418, Tc=Ts, Kt=0)
        #self.angleController =PID(Kp=7.8455, Ki = 9.8703, Kd=0, Tc=Ts,Kt=0)

    def update(self, phi, phi_dot, x, y, x_dot, y_dot, x_des, y_des, points):

        # U_AO velocity vector

        # U_GTG velocity vector

        # clip U_AO and U_GTG to 2m/s

        # blend U_AO and U_GTG to compute ux and uy
        ux = 2
        uy=1
        
        # compute the desired angle and desired thrust
        '''
        write the code to convert ux and uy into desired_angle (i.e., ϕdes ) and
        desired_thrust (i.e., ucol ). Use proportional gain 0.5 for ϕdes and 10
        for ucol .
        '''
        Kphi = .5
        Kthr= 10
        
        x_dot_des = ux
        x_dot_error = x_dot_des - x_dot
        desired_angle = .5*x_dot_error 
        
        y_dot_des = uy 
        y_dot_error= y_dot_des - y_dot
        desired_thrust = Kthr*y_dot_error + self.m*self.gravity/np.cos(phi)
        
        # update the angle rate and angle controllers to compute the desired differential thrust
        rin = self.angleController.update( desired_angle, phi )         
        udif = self.angleRateController.update(rin, phi_dot)
        # compute actuator commands

        motor1_desired_thrust = .5*(desired_thrust + udif)
        motor2_desired_thrust = .5*(desired_thrust - udif)
        return motor1_desired_thrust, motor2_desired_thrust

