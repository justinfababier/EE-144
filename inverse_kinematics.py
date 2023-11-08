import numpy as np
from math import pi, cos, sin, atan, atan2, sqrt, acos

def inverse_kinematics(position):
    # input: the position of end effector [x, y, z]
    # output: joint angles [joint1, joint2, joint3]
    
    link1z = 0.065
    link2z = 0.039
    link3x = 0.050
    link3z = 0.150
    link4x = 0.150
    x = position[0]
    y = position[1]
    z = position[2]

    # Joint 1 angle (joint1)
    joint1 = atan2(y, x)

    # Distance from origin to wrist
    r = sqrt(x**2 + y**2) - link3x

    # Distance from wrist to end effector
    d = z - link1z - link3z

    # Law of cosines to find angle between link2 and link3
    cos_gamma = (r**2 + d**2 - link2z**2 - link4x**2) / (2 * link2z * link4x)
    sin_gamma = sqrt(1 - cos_gamma**2)

    # Joint 2 angle (joint2)
    joint2 = atan2(d, r) - atan2(link4x * sin_gamma, link2z + link4x * cos_gamma)

    # Joint 3 angle (joint3)
    joint3 = acos(cos_gamma)

    return [joint1, joint2, joint3]
