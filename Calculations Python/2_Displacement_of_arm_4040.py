# -*- coding: utf-8 -*-
"""
Created on Tue Oct 20 19:40:14 2020

@author: peppi
"""

L_arm2 = 1200  # Length Vertical pole (mm)
E = 70000  # Elasticity modulus (N/mm^2)
I = 72604  # mm^4
sigma_max = 200  # Or RP0.2 (N/mm^2)
y_max = 20  # (mm)
Fg_gripper = 123.6  # N
Faz_gripper = 11.3
Fg_arm = 44.7
Faz_arm = 4.07
Mx_gripper = 7.6

F1 = Fg_arm + Faz_arm
F2 = Fg_gripper + Faz_gripper

displacement = (Mx_gripper * L_arm2 ** 2) / (2 * E * I) + (F2 * (L_arm2 ** 3)) / (3 * E * I) + (
            F1 * (0.5 * L_arm2) ** 2) / (2 * E * I) + (F1 * (0.5 * L_arm2) ** 3) / (3 * E * I)

M_max_break = (I * sigma_max) / y_max
F_max_break = M_max_break / L_arm2

print("Displacement=", displacement, 'mm')
print("M_max_break=", M_max_break / 1000, "Nm")
print("F_max_break=", F_max_break, "N")