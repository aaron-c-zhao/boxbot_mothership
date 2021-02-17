# -*- coding: utf-8 -*-
"""
Created on Thu Sep  3 19:41:10 2020

@author: peppi
"""
from sympy import symbols, Eq, solve



L1 = 1200               #Length Horizontal pole (mm)
L2 = 2000              #Length Vertical pole (mm)         
E = 70000               #Elasticity modulus (N/mm^2)
sigma_max = 200         #Or RP0.2 (N/mm^2)
y_max = 40              #(mm)
F =100                  #N
delta = 1               #maximaal toegestane doorbuiging (mm)



M = F*L1





I = symbols('I')
eq1 = Eq(((M*L2)/(E*I))*L1 + (F*(L1**3))/(3*E*I) - delta)


I = solve(eq1)
print(I)

F_max_break = ((I[0]*sigma_max)/y_max)/L1


#delta = ((M*L2)/(E*I))*L1 + (F*(L1**3))/(3*E*I)         #angle*L1 + displacement of L    




print("F_max_break=" ,F_max_break,"N")