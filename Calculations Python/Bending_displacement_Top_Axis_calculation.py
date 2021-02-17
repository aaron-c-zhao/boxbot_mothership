# -*- coding: utf-8 -*-
"""
Created on Sat Oct 17 17:10:27 2020

@author: peppi
"""
import numpy as np

r = 0.04 #m
m_total_arm = 15 #kg
g= 9.81 #m/s^2
L =0.1 #m
E_steel = 210*(10**9)


T = g*m_total_arm
I_shaft= (np.pi*(r**4))/4


delta = (2*T*L**3)/(48*E_steel*I_shaft)
print('Doorbuiging in de as is:', delta*(10**3), 'mm')