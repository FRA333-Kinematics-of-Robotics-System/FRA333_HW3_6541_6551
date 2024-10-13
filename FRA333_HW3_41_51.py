# file สำหรับเขียนคำตอบ
# ในกรณีที่มีการสร้าง function อื่น ๆ ให้ระบุว่า input-output คืออะไรด้วย
from HW3_utils import FKHW3
from math import pi

import numpy as np

'''
ชื่อ_รหัส(ex: ธนวัฒน์_6541)                                                        
1. พบเพชร_6541
2. วัสยศ_6551
3.
'''

q_initial = np.array([0.0, 0.0, 0.0])
q_singularity = np.array([0.0, pi/4, pi/4])

w_initial = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] #(Fx, Fy, Fz, Tx, Ty, Tz)

#=============================================<คำตอบข้อ 1>======================================================#
#code here
def endEffectorJacobianHW3(q:list[float])->list[float]:
    R,P,R_e,p_e = FKHW3(q)
    
    ''' P =
    [ 0       0       0.425           0.89943]
    [ 0       0       -5.2047489e-17  0.109  ]
    [ 0.0892  0.0892  0.0892          -0.0038] 
    '''
    
    J = np.zeros((6, len(q)))
    
    for i in range(len(q)):
        P_i = P[:, i]
        Z_i = R[:, 2, i]

        # Linear velocity part
        J[:3, i] = (np.cross(Z_i, p_e - P_i)) @ R_e

        # Angular velocity part
        J[3:, i] = Z_i @ R_e
    
    return J
#==============================================================================================================#

#=============================================<คำตอบข้อ 2>======================================================#
#code here
def checkSingularityHW3(q:list[float])->bool:
    J = endEffectorJacobianHW3(q)
    J_Reduce = J[:3,:]
    
    manipularity = abs(np.linalg.det(J_Reduce))
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
    
    return tau
#==============================================================================================================#

print(endEffectorJacobianHW3(q_initial))
print(checkSingularityHW3(q_singularity))
print(computeEffortHW3(q_initial, w_initial))