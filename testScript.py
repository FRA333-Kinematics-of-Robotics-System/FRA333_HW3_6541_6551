# file สำหรับตรวจคำตอบ
# ในกรณีที่มีการสร้าง function อื่น ๆ ให้ระบุว่า input-output คืออะไรด้วย

from FRA333_HW3_6541_6551 import endEffectorJacobianHW3,checkSingularityHW3,computeEffortHW3
from spatialmath import SE3
from math import pi

import roboticstoolbox as rtb
import numpy as np

'''
ชื่อ_รหัส(ex: ธนวัฒน์_6541)                                                        
1. พบเพชร_6541
2. วัสยศ_6551
'''

d_1 = 0.0892
a_2 = 0.425
a_3 = 0.39243
d_4 = 0.109
d_5 = 0.093
d_6 = 0.082
q_initial = np.array([0.0, 0.0, 0.0])
# q_singularity_1 = np.array([0.0, pi/2, 0.0])
q_singularity_1 = np.array([-3.14159265 ,-1.57079633 , 2.9511931])
q_singularity_2 = np.array([0.0, pi/6, 3.13])

w_initial = [1.0, 2.0, 3.0, 0.0, 0.0, 0.0] #(Fx, Fy, Fz, Tx, Ty, Tz)

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
# print(robot)
#===========================================<ตรวจคำตอบข้อ 1>====================================================#
#code here

def CheckJacobian(q:list[float])->list[float]:

    j = robot.jacobe(q) # หาค่า jacobian จาก robot และ Roboticstoolbox
    return j

#==============================================================================================================#
#===========================================<ตรวจคำตอบข้อ 2>====================================================#
#code here

def CheckSingularity(q:list[float])->bool:

    j = robot.jacobe(q)
    J_Reduce = j[:3,:] # ใช้เฉพาะ 3 แถวแรก 
    det_J = np.linalg.det(J_Reduce) # หา det ของ jacobian

    singularity = abs(det_J) < 1e-3  # เปรียบเทียบ determinant 
    if singularity: # เช็คว่าเป็น singularity ไหม
        singularity = True
    else:   
        singularity = False
    # robot.plot(q, block=True) # plot แขนกล
    return singularity


#==============================================================================================================#
#===========================================<ตรวจคำตอบข้อ 3>====================================================#
#code here
def CheackEffort(q:list[float], w:list[float])->list[float]:
    
    j = robot.jacobe(q)
    #วิธีที่ 1
    effort_robotics_toolbox = robot.pay(w , q , j) # หาค่า tau จากคำสั่ง robot.pay

    #วิธีที่ 2 หา tau จากสูตร τ=JT⋅w
    j_Trans = np.transpose(j) # Transpose jacobian
    tau = j_Trans @ w  # หาค่า tau
    return(tau,effort_robotics_toolbox)

#ค่า tau จากทั้ง 2 วิธีอาจตรงข้ามกัน (สลับ + -) เนื่องจากมีการ reference frame ที่ต่างกัน

#==============================================================================================================#


print("------------------------------------------------<CheckJacobian>-------------------------------------------------")

J = endEffectorJacobianHW3(q_initial)
j = CheckJacobian(q_initial)

print('Manual :')
print(J)
print(' ')
print('Roboticstoolbox :')
print(j)

print("-----------------------------------------------<CheckSingularity>-----------------------------------------------")

print('Manual :')
print(checkSingularityHW3(q_singularity_1))
print(' ')
print('Roboticstoolbox :')
print(CheckSingularity(q_singularity_1))

print("-------------------------------------------------<CheckEffort>--------------------------------------------------")

T = computeEffortHW3(q_initial, w_initial)
t = CheackEffort(q_initial, w_initial)

print('Manual :')
print(T)
print(' ')
print('Roboticstoolbox :')
print(t)