# HW3 Robotics: Jacobian and Singularity Analysis

This project contains Python functions to calculate the Jacobian matrix of a robot's end-effector, check for singularities, and compute the required joint torques for a given wrench applied to the end-effector. The code relies on a provided utility function `FKHW3` for forward kinematics and uses **Robotics Toolbox for Python** for validation.

## Team Members
1. Poppeth Pethchamli
2. Vasayos Tosiri

## Prerequisites

Ensure you have the following installed:
- Python 3.x
- `numpy` library for matrix operations
- `math` module for handling pi and other mathematical operations
- `roboticstoolbox-python` for validating the results (you can install this using `pip install roboticstoolbox-python`)
- `spatialmath-python` for transformations (you can install this using `pip install spatialmath-python`)

### Files Provided
- `HW3_utils.py`: Contains the `FKHW3` function for calculating forward kinematics. This file must be in the same directory as the script.
- `FRA333_HW3_41_51.py`: Contains the main functions for calculating the Jacobian, checking for singularities, and computing joint efforts.
- `testScript.py`: Script to verify the correctness of the main functions using Robotics Toolbox for Python.
- `singularity_finder.py`: A script that detects singular configurations of a 3-degree-of-freedom (3DOF) robot.

### Our Robot

![robot](pic1.png)

This is our Robot MDH parameters.
- **d_1** = 0.0892 

- **a_2** = 0.425

```bash
DHRobot: 3DOF_Robot, 3 joints (RRR), dynamics, modified DH parameters

                                ┌────────┬───────┬────────────┬──────┐
                                │  aⱼ₋₁  │ ⍺ⱼ₋₁  │     θⱼ     │  dⱼ  │
                                ├────────┼───────┼────────────┼──────┤
                                │   0.0  │  0.0° │  q1 + 180° │  d_1 │
                                │   0.0  │ 90.0° │         q2 │  0.0 │
                                │  -a_2  │  0.0° │         q3 │  0.0 │
                                └────────┴───────┴────────────┴──────┘
```

### Functions Overview

1. **`endEffectorJacobianHW3(q: list[float]) -> list[float]`**

    - **Input**: 
        - `q`: A list of joint angles representing the robot's configuration.
    - **Output**: 
        - Returns a 6xN Jacobian matrix, where the first three rows represent the linear velocity and the last three rows represent the angular velocity of the end-effector.
    
    - **Description**: 
        - This function calculates the Jacobian matrix for the robot based on its current configuration. It uses the forward kinematics results to compute the linear and angular velocity components.

    - **Jacobian Formula**:

2. **`checkSingularityHW3(q: list[float]) -> bool`**

    - **Input**: 
        - `q`: A list of joint angles representing the robot's configuration.
    - **Output**: 
        - Returns `True` if the robot is in a singularity configuration, otherwise `False`.
    
    - **Description**: 
        - This function checks if the robot's current configuration is at or near a singularity by computing the determinant of the Jacobian's translational component.

    - **Singularity Calculation**:

        To find singularities in a robot's motion using the Jacobian, we use this following relationship.

        <h3 align="center">
          || det (J<sup>*</sup>(q)) || &lt; &epsilon;
        </h3>
        
        Where:
        - $J^*$ is the **Jacobian** matrix of the robot that already **reduce**.
        - $q$ is the list of joint angles representing the robot's configuration.
        - $\epsilon$ is the **small threshold value** that helps determine whether the robot is in or near a singular configuration.

3. **`computeEffortHW3(q: list[float], w: list[float]) -> list[float]`**

    - **Input**: 
        - `q`: A list of joint angles representing the robot's configuration.
        - `w`: A list of 6 elements representing the wrench (forces and torques) applied at the end-effector.
    - **Output**: 
        - Returns a list of joint torques required to resist the applied wrench.
    
    - **Description**: 
        - This function computes the joint torques needed to resist a given wrench applied to the end-effector using the transpose of the Jacobian matrix.  

    - **Effort Calculation**:

        To compute the effort (joint torques or forces) for a 3 DOF robot using the Jacobian, we use this following relationship.

        <h3 align="center">
          &tau; = J^T &times; w
        </h3>

        Where:
        - $\tau$ is the vector of joint torques (effort).
        - $J$ is the **Jacobian** matrix of the robot.
        - $w$ is the **wrench** (forces and torques) applied at the end-effector.

### Sample Usage

```python
from FRA333_HW3_41_51 import endEffectorJacobianHW3, checkSingularityHW3, computeEffortHW3
from math import pi

import numpy as np

# Define initial joint configuration and wrench
q_initial = np.array([0.0, 0.0, 0.0])
w_initial = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])  # No force or torque applied

# Calculate the Jacobian at the initial configuration
print("--------------------<Jacobian>--------------------")
print(endEffectorJacobianHW3(q_initial))

# Check if the robot is at a singularity
q_singularity = np.array([0.0, pi/6, 3.13])

print("--------------------<Singularity>--------------------")
print(checkSingularityHW3(q_singularity))

# Compute the joint torques for the given wrench
print("--------------------<Zero Effort>--------------------")
print(computeEffortHW3(q_initial, w_initial)) #return [0.0, 0.0, 0.0]

w = np.array([0.0, 0.0, 10.0, 0.0, 0.0, 0.0]) #with force

print("--------------------<with Effort>--------------------")
print(computeEffortHW3(q_initial, w))
```

### Validation Script

The provided `testScript.py` verifies the correctness of the manually calculated Jacobian, singularity check, and effort computation using **Robotics Toolbox for Python**.
### Robot's MDH Parameters Define for **Robotics Toolbox**

![robottoolbox](all_dim_robot.png)

```python
#This parameters from HW3_utils.py file.
d_1 = 0.0892
a_2 = 0.425
a_3 = 0.39243
d_4 = 0.109
d_5 = 0.093
d_6 = 0.082

robot = rtb.DHRobot(
    [
        rtb.RevoluteMDH(alpha = 0.0     ,a = 0.0      ,d = d_1    ,offset = pi ),
        rtb.RevoluteMDH(alpha = pi/2    ,a = 0.0      ,d = 0.0    ,offset = 0.0),
        rtb.RevoluteMDH(alpha = 0.0     ,a = -a_2     ,d = 0.0    ,offset = 0.0),
    ],
    tool = SE3([
    [0, 0, -1, -(a_3 + d_6)],
    [0, 1, 0, -d_5],
    [1, 0, 0, d_4],
    [0, 0, 0, 1]]),
    name = "3DOF_Robot"
    )
```

### Validation Functions

1. **`CheckJacobian(q: list[float]) -> list[float]`**

    - **Input**: 
        - `q`: A list of joint angles representing the robot's configuration.
    - **Output**: 
        - Returns a 6xN Jacobian end-effector matrix from **robotics toolbox**.

    - **Description**: 
        - Uses Robotics Toolbox to calculate the Jacobian matrix for comparison with the manually computed Jacobian.
    
2. **`CheckSingularity(q: list[float]) -> bool`**

    - **Input**: 
        - `q`: A list of joint angles representing the robot's configuration.
    - **Output**: 
        - Returns `True` if the robot is in a singularity configuration, otherwise `False`.

    - **Description**: 
        - Uses the Jacobian determinant to check for singularities and compares it with the result from the manual method.

3. **`CheackEffort(q: list[float], w: list[float]) -> list[float]`**

    - **Input**: 
        - `q`: A list of joint angles representing the robot's configuration.
        - `w`: A list of 6 elements representing the wrench (forces and torques) applied at the end-effector.
    - **Output**: 
        - Returns a list of joint torques required to resist the applied wrench from both two methods.

    - **Description**: 

        Computes the joint torques based on a given wrench using two methods:

        - Using the `robot.pay()` method from Robotics Toolbox.
        - Using the formula `τ = JT ⋅ w` for comparison with the manual method.
