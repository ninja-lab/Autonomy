# Differential drive robot controller. Implements PID on two motors and pure pursuit + AO. 
# Clips desired speeds to 2 m/s, which somewhat avoids motor saturation at steady-state
# Copyright 2024, Prof. Hamid Ossareh @ University of Vermont

from PID import PID
import numpy as np
#from numpy.linalg import norm 

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
        self.target = np.array([0, 0])

        # motor controllers are PI with anti-windup
        self.leftWheelController = PID(Kp=1/3, Ki=10/3, Kd=0, Ts = Ts, umax = 10, umin = -10, Kaw = 1)
        self.rightWheelController = PID(Kp=1/3, Ki=10/3, Kd=0, Ts = Ts, umax = 10, umin = -10, Kaw = 1)

    # Update function. Goal: go to goal
    # measurements: left and right motor speeds, robot heading and x/y positions
    # outputs: left and right motor voltages
    def update(self, omega_l, omega_r, phi, x, y, path, points):

        # Find the point on a line segment closest to a given point.
        # P is the given point. A is the starting point of the segment.
        # B is the end point of the segment.
        def get_normal_point(P, A, B):
            a = P - A
            b = B-A
            c = A + np.dot(a, b)*b/(norm(*b)**2 )
            minX = min(A[0], B[0])
            maxX = max(A[0], B[0])
            minY = min(A[1], B[1])
            maxY = max(A[1], B[1])
            if c[0] >= minX and c[0] <= maxX and c[1] >= minY and c[1] <= maxY:
                return c 
            elif norm(*a) < norm(*(P-B)):
                return A
            else:
                return B 
            

        # Find the target, a look-ahead distance ahead of the point closest 
        #to the robot on the path
        def find_target(path, P):
            '''
            first use the get_normal_point function to find the points closest 
            to P on every line segment comprising the path.
            
            Then, among the N − 1 points that are computed, choose the one
            closest to P (use the “norm” function to calculate distances)
            
            Finally, compute the target (i.e., the output of the function) to be 
            the point on the path that is 2.5m ahead of Q (towards the endpoint
            of the path). If target is beyond the final endpoint of
            the path, then the target must be the endpoint.
            N waypoints 
            N-1 segments, N-1 closest points, N-1 norms
            '''
            look_ahead = 2.5
            
            N = path.shape[1]
            closest_points = np.zeros((2, N-1))
            norms = np.zeros(N-1)
            for i in range(N-1):
                closest_points[:,i] = get_normal_point(P, path[:, i+1], path[:,i]) 
                norms[i] = norm(*(P-closest_points[:,i]))            
            #if closest point is in the first segment, Qindex is 0 and 
            #next endpoint is path[Qindex + 1]
            #maximum of Qindex is N-1 
            Qindex = np.argmin(norms)
            Q = closest_points[:,Qindex]
            Q_endpoint = path[:, Qindex + 1]
            distance_to_endpoint = norm(*(Q_endpoint - Q))
            if distance_to_endpoint >= look_ahead:
                #there is enough room left on this path
                #add 2.5 to current direction 
                
                direction = path[:, Qindex+1] - path[:, Qindex] 
                direction_unit =  direction/norm(*direction)
                target =  closest_points[:,Qindex] + direction_unit*2.5
            else: 
                #need to wrap towards next path segment, but only if 
                #there is a next path segment 
                diff = look_ahead - distance_to_endpoint 
                if Qindex == N - 2 :
                    target =  path[:, -1]
                else:
                    direction = path[:,Qindex+2] - path[:,Qindex+1 ]
                    direction_unit =  direction/norm(*direction)
                    target =  path[:,Qindex+1] + direction_unit*diff
            return target 

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
        self.target = find_target(path, np.array([x, y]))
        x_des, y_des = self.target[0], self.target[1]
        
        
        U_GTG = self.K_e*np.array([x_des - x, y_des - y])

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
    

