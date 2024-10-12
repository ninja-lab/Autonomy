#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Oct  3 20:58:03 2024

@author: erik
"""
import math 
class PID: 
    def __init__ (self, Kp=1, Ki=1, Kd=1, Tf=.1, Tc=0.01, Kt=1,
                  sat_min= -math.inf, sat_max = math.inf, initial_condition=0): 
        self.Kp = Kp 
        self.Ki = Ki 
        self.Tc = Tc 
        self.Kd = Kd 
        self.Tf = Tf
        self.sat_max = sat_max
        self.sat_min = sat_min 
        self.Kt = Kt
        self.integrator_state = initial_condition 
        self.e_previous = 0
        self.ud = 0
        self.es_previous = 0 
        
    def update(self, r, y): 
        error = r - y 
        self.ud = (1/self.Tf)*(error - self.e_previous-(self.Tc-self.Tf)*self.ud)
        self.integrator_state = self.integrator_state + \
            self.Tc*(self.Ki*error + self.Kt*self.es_previous)
 
        self.ctl_cmd = self.Kp*error + self.integrator_state + self.Kd*self.ud
        self.ctl_cmd_sat = max( min (self.ctl_cmd, self.sat_max), self.sat_min)
        self.es_previous = self.ctl_cmd_sat - self.ctl_cmd
        self.e_previous = error
        return self.ctl_cmd_sat