#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Oct  3 21:13:14 2024

@author: erik
"""
a = 0.1 + 0.2
if (a==0.3):
    print('yes')
else:
    print('No')
    
    
if (abs(a-0.3) < 1e-16):
    print('yes')
else:
    print('No')