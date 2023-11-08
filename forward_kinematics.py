import numpy as np
from math import pi, cos, sin
import modern_robotics as mr

def forward_kinematics(joints):
    # input: joint angles [joint1, joint2, joint3]
    # output: the position of end effector [x, y, z]
    
    link1z = 0.065
    link2z = 0.039
    link3x = 0.050
    link3z = 0.150
    link4x = 0.150
    joint1 = joints[0]
    joint2 = joints[1]
    joint3 = joints[2]
    
    # Define DH parameters
    theta = [joint1, joint2, joint3]
    d = [0, link2z, 0, 0]
    a = [0, link3x, link4x, 0]
    alpha = [pi/2, 0, 0, 0]
    
    # Initialize transformation matrix to identity
    T = np.eye(4)
    
    # Compute transformation matrices
    for i in range(3):
        A_i = mr.MatrixExp6(mr.VecTose3([a[i], d[i], link1z, theta[i]]))
        T = np.dot(T, A_i)
    
    # Extract end effector position from the transformation matrix
    x = T[0, 3]
    y = T[1, 3]
    z = T[2, 3]

    return [x, y, z]
