import numpy as np
from math import pi, cos, sin, atan2, sqrt, acos

def inverse_kinematics(position):
    """
    Calculate the joint angles for a given end effector position.
    
    Parameters:
    position (list): A list of [x, y, z] coordinates of the end effector.
    
    Returns:
    list: A list of joint angles [joint1, joint2, joint3].
    """
    # Link lengths and positions
    link1z = 0.065
    link2z = 0.039
    link3x = 0.050
    link3z = 0.150
    link4x = 0.150
    
    # End effector position
    x = position[0]
    y = position[1]
    z = position[2]
    
    # Initialize variables
    k = 0.1581  # Length given in image

    # Calculate joint1 angle
    joint1 = atan2(y, x)
    
    # Distance calculations
    r = sqrt(x**2 + y**2)
    a = sqrt(r**2 + (z - link1z - link2z)**2)
    
    # Angle calculations
    gamma = acos(r / a)
    beta_1 = acos((a**2 + k**2 - link4x**2) / (2 * a * k))
    alpha = acos(link3z / k)
    
    # Calculate joint2 angle
    joint2 = (pi / 2) - alpha - beta_1 - gamma
    
    # More angle calculations
    beta_2 = acos((link4x**2 + k**2 - a**2) / (2 * k * link4x))
    eta = atan2(link3z, link3x)
    
    # Calculate joint3 angle
    joint3 = beta_2 - (pi - eta)
    
    return [joint1, joint2, joint3]

# Example usage
position = [0.1, 0.1, 0.1]
print(inverse_kinematics(position))
