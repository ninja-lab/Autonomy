#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Aug 31 07:03:16 2024

@author: erik
"""
import re 
pattern1 = re.compile('autonomy', re.IGNORECASE)
pattern2 = re.compile('exit', re.IGNORECASE)
for i in range(3):
    ans = input("Please enter the title of this course:")

    if pattern1.search(ans):
        print('Correct')
        exit() 
    elif pattern2.search(ans):
        exit()
    

    