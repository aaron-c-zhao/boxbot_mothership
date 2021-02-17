import numpy as np

d = 0.025                                               #m (diameter of the shaft)
Fg_gripper = 123.606                                    #N (Gravity force of the gripper)
Fg_arm = 57.54                                          #N (Gravity force of the arm)
L_stand = 2                                             #m  (Length of crane tower)
L_arm1 = 1.2                                            #m  (length of crane arm
l = 0.03                                                #m  (length of shaft)
E_Fe = 210*(10**9)                                      #Pa (Elasticity modulus steel)

M_load = Fg_gripper*L_arm1 + Fg_arm*0.5*L_arm1                                      #Nm (Load on the shaft)
I = (np.pi/4)*((d/2)**4)

theta = (M_load*l)/(E_Fe*I)                              #rad (angular displacement of the shaft)
delta = theta*(np.sqrt((L_stand**2)+(L_arm1**2)))       #m (displacement of the end of the arm due to the angular displacement of the shaft)

print('theta=', theta, 'displacement angle of the shaft')
print('delta=', delta, 'total displacement of the end of the arm due to the angle theta')
