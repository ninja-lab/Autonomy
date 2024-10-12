# The drone controller
# Copyright 2024, Prof. Hamid Ossareh @ University of Vermont

from PID import PID
import pdb
import math
class Controller:
    def __init__(self, Ts):
        # Nominal parameters that we can use for the quadcopter.
        self.Ts = Ts
        self.m = 2          # mass in kg
        self.J = .05       # mass moment of inertia
        self.motorTC = 0.2  # motor time constant
        self.l = 0.15       # length of each arm
        self.gravity = 9.81 # acceleration due to gravity

        # Class: initialize the two PID controllers here
        #1/(60*5) = 0.0033333 sec     Tf = .00021876
        #self.angleRateController = PID(Kp = 30.5614, Ki = 78.4266, Kd=2.6188, Tf = .00021876, Tc=Ts, Kt=1.3, 
        #                               sat_min= -10, sat_max = 10)
        #self.angleRateController = PID(Kp = 30.5614, Ki = 78.4266, Kd=2.6188, Tf = .00021876, Tc=Ts, Kt=0)
        # Kp, Ki, Kd, N, Ts, umax, umin, Kt)
        #self.angleRateController = PID(30.5614,78.4266,2.6188,.00021876, Ts, 10, -10, 1.3)
        #self.angleRateController = PID(Kp = 10, Ki = 0, Kd=0, Tf = 1, Tc=Ts, Kt=0)
        self.angleRateController = PID(Kp = 13.6, Ki = 0, Kd=1.27, Tf = .00418, Tc=Ts, Kt=0)


        self.angleController =PID(Kp=7.8455, Ki = 9.8703, Kd=0, Tc=Ts,Kt=0)
        #self.angleController =PID(7.8455, 9.8703, 0, 1, Ts, math.inf, -math.inf,  Kt=0)
        #self.angleController =PID(7.8455, 9.8703, 0, 1, Ts, 10, -10,  Kt=1.3)
        #self.angleController =PID(Kp=1.7, Ki = .36, Kd=0, Kt=0, Tc=Ts)

    def update(self, phi, phi_dot, desired_angle, desired_thrust):
        # update the PID controllers and return the motor commands
        '''
        the user has direct control over ucol but not udif . Instead, the user
        can specify the desired angle, rϕ , which your drone needs to track. The user 
        specifies these two quantities (ucol and rϕ ) by moving their mouse cursor
        on the screen when the pygame simulation is running. The controller calculates
        udif and, in turn u1 and u2 , to ensure that ϕ(t) → rϕ .
        desired_thrust = ucol 
        
        '''
        
        rin = self.angleController.update( desired_angle, phi )         
        udif = self.angleRateController.update(rin, phi_dot)
        motor1_desired_thrust = .5*(desired_thrust + udif)
        motor2_desired_thrust = .5*(desired_thrust - udif)
        #breakpoint()
        return motor1_desired_thrust, motor2_desired_thrust

