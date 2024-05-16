import numpy as np
from math import pi, cos, sin, atan, atan2, sqrt, acos

def inverse_kinematics(position):
    # input: the position of end effector [x, y, z]
    # output: joint angles [joint1, joint2, joint3]
    # add your code here to complete the computation

    link1z = 0.065
    link2z = 0.039
    link3x = 0.050
    link3z = 0.150
    link4x = 0.150
    x = position[0]
    y = position[1]
    z = position[2]

    alpha = 0.0
    gamma = 0.0
    eta = 0.0
    r = 0.0
    a = 0.0
    k = 0.1581  # Length given in image
    beta_1 = 0.0
    beta_2 = 0.0

    joint1 = 0.0
    joint2 = 0.0
    joint3 = 0.0

    joint1 = atan2(y,x)

    r = sqrt(pow(x,2) + pow(y,2))

    a = sqrt(pow(r,2)+pow((z-link1z-link2z),2))

    gamma = acos(r/a)

    beta_1 = acos((pow(a,2)+pow(k,2)-pow(link4x,2))/(2*a*k))

    alpha = acos(link3z/k)

    joint2 = (pi/2) - alpha - beta_1 - gamma

    beta_2 = acos((pow(link4x,2)+pow(k,2)-pow(a,2))/(2*k*link4x))

    eta = atan(link3z/link3x)

    joint3 = beta_2 - (pi - eta)
    
    return [joint1, joint2, joint3]
