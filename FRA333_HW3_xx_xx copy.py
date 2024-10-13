# file สำหรับเขียนคำตอบ
# ในกรณีที่มีการสร้าง function อื่น ๆ ให้ระบุว่า input-output คืออะไรด้วย
from spatialmath import SE3
from HW3_utils import FKHW3
from math import pi

import roboticstoolbox as rtb
import numpy as np

'''
ชื่อ_รหัส(ธนวัฒน์_6461)
1.
2.
3.
'''
d_1 = 0.0892
a_2 = 0.425
a_3 = 0.39243
d_4 = 0.109
d_5 = 0.093
d_6 = 0.082
q_initial = np.array([0.0, 0.0, 0.0])
q_singularity = np.array([0.0, pi/4, pi/2])

w_initial = [0.0, 0.0, 10.0, 0.0, 0.0, 0.0] #(Fx, Fy, Fz, Tx, Ty, Tz)

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

#=============================================<คำตอบข้อ 1>======================================================#
#code here
def endEffectorJacobianHW3(q:list[float])->list[float]:
    
    R,P,R_e,p_e = FKHW3(q)
    
    '''
    [ 0       0       0.425           0.89943]
    [ 0       0       -5.2047489e-17  0.109  ]
    [ 0.0892  0.0892  0.0892          -0.0038] 
    '''
    
    # print(P)
    J = np.zeros((6, len(q)))
    
    for i in range(len(q)):
        # print("-------------------")
        P_i = P[:, i]
        Z_i = R[:, 2, i]
        # print(Z_i)
        # Linear velocity part
        J[:3, i] = (np.cross(Z_i, p_e - P_i)) @ R_e

        # Angular velocity part
        J[3:, i] = Z_i

    # print(J)
    # print("--------------------------------------------------------")
    # print(robot.jacobe(q))
    
    # print(p_e)
    
    # robot.plot(q, block=True)
    return J
    # Jcob = robot.jacob0(q)
    # pass
#==============================================================================================================#

#=============================================<คำตอบข้อ 2>======================================================#
#code here
def checkSingularityHW3(q:list[float])->bool:
    J = endEffectorJacobianHW3(q)
    # print(J)
    # print("--------------------")
    J_Reduce = J[:3,:]
    
    # print(J_Reduce)
    
    manipularity = np.linalg.det(J_Reduce)
    singularity = False
    
    if manipularity <  1e-3:
        singularity = True
        
    return singularity
#==============================================================================================================#

#=============================================<คำตอบข้อ 3>======================================================#
#code here
def computeEffortHW3(q:list[float], w:list[float])->list[float]:
    J = endEffectorJacobianHW3(q)

    J_Trans = np.transpose(J)
    
    tau = J_Trans @ w
    # print(effort)
    return tau
#==============================================================================================================#

# endEffectorJacobianHW3(q_initial)
# print(endEffectorJacobianHW3(q_initial))
# print(checkSingularityHW3(q_singularity))
# print(computeEffortHW3(q_initial, w_initial))