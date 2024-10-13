import numpy as np
import roboticstoolbox as rtb
from spatialmath import SE3
from math import pi

# Define robot parameters
d_1 = 0.0892
a_2 = 0.425
a_3 = 0.39243
d_4 = 0.109
d_5 = 0.093
d_6 = 0.082

# Define the robot using DH parameters
robot = rtb.DHRobot(
    [
        rtb.RevoluteMDH(alpha=0.0     ,a=0.0      ,d=d_1    ,offset=pi),
        rtb.RevoluteMDH(alpha=pi/2    ,a=0.0      ,d=0.0    ,offset=0.0),
        rtb.RevoluteMDH(alpha=0.0     ,a=-a_2     ,d=0.0    ,offset=0.0),
    ],
    tool=SE3([
    [0, 0, -1, -(a_3 + d_6)],
    [0, 1, 0, -d_5],
    [1, 0, 0, d_4],
    [0, 0, 0, 1]]),
    name="3DOF_Robot"
)

# Function to check singularity
def is_singular(q):

    J = robot.jacob0(q)     # Calculate Jacobian matrix
    
    det_J = np.linalg.det(J[:3, :3])  # Calculate determinant of Jacobian
    
    return np.abs(det_J) < 1e-6, det_J     # Check if determinant is near zero 

# Function to search for all q that result in singularity
def find_all_singularities():
    singular_qs = []  # List to store singular configurations

    # Loop for find  q1, q2, q3 100 value
    for q1 in np.linspace(-pi, pi, 100):
        for q2 in np.linspace(-pi/2, pi/2, 100):
            for q3 in np.linspace(-pi, pi, 100): 
                q = np.array([q1, q2, q3])
                singular, det_J = is_singular(q)
                if singular:
                    singular_qs.append((q, det_J))
                    print(f"Singular configuration found: q = {q}, Determinant = {det_J}")
    
    return singular_qs

singular_q_list = find_all_singularities()
if singular_q_list:
    print(f"\nTotal {len(singular_q_list)} singular configurations found.")
else:
    print("No singular configurations found.")