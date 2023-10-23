# -*- coding: utf-8 -*-
"""

About the script:
An exmple on controlling KUKA iiwa robot from
Python3 using the iiwaPy3 class

Created on Tue Oct  1 11:56:31 2019
Modified on 3rd-Jan-2021

@author: Mohammad Safeea

Test PTP motion class
"""

import math
import time

import numpy as np

from iiwaPy3 import iiwaPy3
from KUKA_IK import KUKA_IK

# Connect to the robot
ip='192.170.10.106'
#ip='localhost'
iiwa=iiwaPy3(ip)
InvKin = KUKA_IK(np.identity(4))
time.sleep(2)

try:

    # #MOVE TO INITIAL POSITION
    # jPos = [math.pi/2,0,0,-math.pi/2,0,math.pi/2,0]
    #
    # print("Moving the robot in joint space to angular position")
    # print(jPos)
    # vRel=[0.1]
    # print("With a relative velocity")
    # print(vRel[0])
    # iiwa.movePTPJointSpace(jPos,vRel)

    P_des = np.array([[1, 0, 0, 0.15], [0, 0, 1, 0.68], [0, -1, 0, 0.74], [0, 0, 0, 1]])
    # TEST MY IK FUNCTION

    # P_des[0, 3]=-0.55
    # P_des[1, 3]=-0.005
    # P_des[2, 3]= 0.68
    w_task = np.array([1, 1, 1, 1, 1, 1])

    jPos = InvKin.InverseKinematics(P_des, w_task)
    Pose_ver = InvKin.getPose(jPos)
    print(Pose_ver)
    print()
    print(jPos * 180 / math.pi)
    relVel = [0.1]

    iiwa.movePTPJointSpace(jPos, relVel)

    Pose = InvKin.getPose(jPos)

    print("Verification: The final pose due to inverse kinematics is:")
    print(Pose)
   

    # INPUT DA TASTIERA
    #print("Enter a Vector whic specifies the EEF coordinates in the form [X,Y,Z,rotX,rotY,rot,Z]")
    #jPos = list(map(float, input("Enter the coordinates:").split()))
 
    #MOVE EEF
    # vel=[30]
    # pos=[0.2 , 0.8, 0.2]
    # iiwa.movePTPLineEefRelBase(pos,vel)

    # print("Current Cartesian pose is")
    # cPos=iiwa.getEEFPos() #EEF position 3 cartesian = 3 orientation
    # print(cPos)

    # iiwa.movePTPLineEefRelEef(pos,vel)


    # Moving in an arc, the arc is in an incliend plane
    # theta=[math.pi/2]
    # k=[1,1,1]
    # vel=[100]
    # c=[cen[0],cen[1],cen[2]]
    # print('Moving on an incliend Arc')
    # iiwa.movePTPArc_AC(theta,c,k,vel)
    # # Go back to beggining point
    # vel=[150] # velocity mm/sec
    # iiwa.movePTPLineEEF(begginingPoint,vel)
    # # Move on an Arc in the XY plange
    # # using functiom movePTPArcXY_AC
    # theta=[1.98*math.pi]
    # c=[cen[0],cen[1]]
    # vel=[150]
    # print('Moving on an incliend Arc parallel to XY plane')
    # iiwa.movePTPArcXY_AC(theta,c,vel)

    time.sleep(2)
    homeVel = [0.1]
    iiwa.movePTPHomeJointSpace(homeVel)


except:
    print('ERROR!')    

iiwa.close()
