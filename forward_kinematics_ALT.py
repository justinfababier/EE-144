import numpy as np
import modern_robotics as mr

def forward_kinematics(joints):
    # input: joint angles [joint1, joint2, joint3]
    # output: the position of end effector [x, y, z]

    # Define link lengths
    link1z = 0.065
    link2z = 0.039
    link3x = 0.050
    link3z = 0.150
    link4x = 0.150
    # Define joint angles
    theta1 = joints[0]
    theta2 = joints[1]
    theta3 = joints[2]

    # Define screw axes
    S1 = np.array([0, 0, 1, 0, 0, 0])
    S2 = np.array([0, 1, 0, -link1z, 0, 0])
    S3 = np.array([0, 1, 0, -(link1z + link2z), 0, 0])

    # Compute the exponential of the screw coordinates
    M1 = mr.MatrixExp6(mr.VecTose3(S1 * theta1))
    M2 = mr.MatrixExp6(mr.VecTose3(S2 * theta2))
    M3 = mr.MatrixExp6(mr.VecTose3(S3 * theta3))

    # Calculate end effector transformation matrix
    T = np.dot(np.dot(np.dot(M1, M2), M3), np.array([[1, 0, 0, link4x],
                                                    [0, 1, 0, 0],
                                                    [0, 0, 1, -(link3x + link3z)],
                                                    [0, 0, 0, 1]]))

    # Extract end effector position from the transformation matrix
    x = T[0, 3]
    y = T[1, 3]
    z = T[2, 3]

    return [x, y, z]
