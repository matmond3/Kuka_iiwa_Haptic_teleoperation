#! /usr/bin/env python3

"""
About the script:
ROS NODE in Python3 for the control of the iiwa LBR14R820.
This node read the position of the End-Effector of an Haptic Device (Omni Geomagic Touch) and mirroring the same
movements in the Joints space through a scaling factor which is specified in the YAML file in kuka/kuka_control/config.
Once the position of the Haptic device is saved in a variable, this is used to compute the joint position of the robot
using the function genInverseKinematics.py and it is finally sent to the kuka for the mirroring.

Created on Mon Jun 07 17:04:31 2021

@author: Matteo Bermond

"""

import yaml
import time
import numpy as np
import rospy
#from getConstrainedPosition import getConstrainedPosition
#from getScaledPosition import getScaledPosition
#from RPY2Quaternions import Rpy2Quaternions

from genInverseKinematics import genInverseKinematics
from setupROS import setupROS
from iiwaPy3 import iiwaPy3
from geometry_msgs.msg import Pose
from QuaternionToRotation import quaternion_rotation_matrix

# LOADING OF WORKSPACE PARAMETERS FROM YAML FILE
with open('/home/matteo/catkin_ws/src/kuka/src/config/kuka_params_rt.yaml') as file:
    info = yaml.full_load(file)
    # print(info)
    # "params" are loaded as a dictionary so to access the informations stored in it you have to use the
    # following syntax: params['key_word'].
    # e.g. params['robot'] return the following string 'LBR14R820'

kukaType = info['robot']
# Connect to the robot
ip = '172.31.1.147'
iiwa = iiwaPy3(ip)
setup = setupROS(info)
InvKin = genInverseKinematics(kukaType, np.identity(4))
scaling_factor = 0.8
j_max = np.deg2rad(2) #setting of the maximum allowed angle for the first movement
max_speed = 0.1
dt = 1

class Callback:
    def __init__(self):
        self.pose = Pose()
        #self.poseWS = Pose()
        self.sub = rospy.Subscriber("Pose4Kuka", Pose, self.callback, queue_size = 10000)
        #self.sub1 = rospy.Subscriber("kuka_wsCenter", Pose, self.cameraCallback, queue_size=10)

    def callback(self, msg):
        self.pose = msg

    # def cameraCallback(self, data):
    #     self.poseWS= data
    #
    # def get_camera_msg(self):
    #     return poseWS

    def get_msg(self):
        """
        Returns the newest Pose data
        """
        return self.pose



try:

    #  Definition of the variable to send to the Haptic device for the Haptic feedback creation
    kukaPos = Pose()
    # read_camera= CameraCallback()
    # WS_data = read_camera.get_camera_msg()
    # WS_Center = np.array([WS_data.position.x, WS_data.position.y, WS_data.position.z])
    # Creation of ROS Node - ROS Publisher
    rospy.init_node('Haptic_EE_listener')
    read_pose = Callback()
    # pub = rospy.Publisher('kuka_position', Pose, queue_size=10000)
    rospy.sleep(2)
    rate = rospy.Rate(1000)

    # Move to home position
    relVel = [0.1]
    w_task = np.array([1, 1, 1, 1, 1, 1])
    jointsHome, T_BT_h, T_BF_h, ws_center = setup.setupWorkspace()
    PoseHome, joints_home = InvKin.InverseKinematics(T_BF_h, jointsHome, w_task)
    iiwa.movePTPJointSpace(joints_home, relVel)
    joints_prev = joints_home
    
    print("\nWs_center:\n")
    print(ws_center)
    print("\nWs_center Camera:\n")
    print(WS_Center)

    EEF_homePose = iiwa.getEEFPos()
    kuka_homePosition = np.array([EEF_homePose[0]/1000, EEF_homePose[1]/1000, EEF_homePose[2]/1000])

    # Are bot Robots in Home position?
    # data_home = read_pose.get_msg()
    # p_home = np.array([data_home.position.x, data_home.position.y, data_home.position.z])
    # The OMNI pose is send as a Pose msg; so I directly obtain the position and the orientation
    # through quaternion of the EEF of the OMNI
    # quat_old = np.array([data_home.orientation.w, data_home.orientation.x, data_home.orientation.y,
    #                      data_home.orientation.z])

    # z_tool = np.array([0, 0, -1])
    # x_tool = - setup.center_line
    # x_tool_norm = x_tool / np.linalg.norm(x_tool)
    # y_tool = np.cross(z_tool, x_tool_norm)
    #
    # T = np.identity(4)
    # T[0:3,0] = x_tool_norm
    # T[0:3,1] = y_tool
    # T[0:3,2] = z_tool
    # T[0:3,3] = kuka_homePosition
    # T[3, 3] = 1

    # rpy = np.array([EEF_homePose[3], EEF_homePose[4], EEF_homePose[5]])
    # quat = Rpy2Quaternions(rpy)
    # p_old = p_home
    # R_centerline = setup.getRotationFromBasis(x_tool,y_tool,z_tool)
    # # R_45 = np.array([[(np.sqrt(2)/2), -(np.sqrt(2)/2), 0], [(np.sqrt(2)/2), (np.sqrt(2)/2), 0], [0, 0, 1]])
    # R_Y = np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]])
    # R_HK = np.array([[0, 0, 1], [0, -1, 0], [1, 0, 0]])
    # R_HK_pos = np.matmul(R_HK, R_centerline)

    iiwa.realTime_startDirectServoJoints()
    value = input("Ready for RT motion?\n")

    while not rospy.is_shutdown():
        # Get the actual pose of the Haptic device
        data = read_pose.get_msg()
        quat = np.array([data.orientation.w, data.orientation.x, data.orientation.y, data.orientation.z])
        # # POSITION FROM HAPTIC DEVICE transformed from METERS to MILLIMETERS
        # pos_haptic_or = np.array([data.position.x, data.position.y, data.position.z])
        #
        # pos_haptic_scaled = scaling_factor * pos_haptic_or
        # pos_haptic = np.matmul(pos_haptic_scaled, R_HK_pos)
        # p_temp = np.array([pos_haptic[0], pos_haptic[1], pos_haptic[2], 1])
        #
        # p = np.matmul(T, p_temp)
        # position = np.array([p[0], p[1], p[2]])

        position = np.array([data.position.x, data.position.y, data.position.z])

        # # Scaled around Workspace center
        # position = getScaledPosition(position, ws_center, scaling_factor)
        # print("\nScale \n")
        # print(position)
        # position_sendHaptic = np.array([position[0], position[1], position[2], 1])
        # pos_kuka_temp = np.matmul(position_sendHaptic, np.linalg.inv(T))
        # pos_kuka = np.array([pos_kuka_temp[0], pos_kuka_temp[1], pos_kuka_temp[2]])

        # # Enforce workspace boundary constrained if necessary
        # position, error, inside, position_sendHaptic = getConstrainedPosition(position, setup)

        # if not inside:
        #     pos_kuka = position_sendHaptic / scaling_factor
        #     kukaPos.position.x = pos_kuka[0]
        #     kukaPos.position.y = pos_kuka[1]
        #     kukaPos.position.z = pos_kuka[2]
        #     pub.publish(kukaPos)

        # pos_kuka = position / scaling_factor
        # kukaPos.position.x = pos_kuka[0]
        # kukaPos.position.y = pos_kuka[1]
        # kukaPos.position.z = pos_kuka[2]
        # pub.publish(kukaPos)

        # if error:
        #     np.disp("Shutting down the application.")
        #     break

        R = quaternion_rotation_matrix(quat)
        # R = np.matmul(R, R_centerline)
        # Construction of the 4-by-4 homogeneous matrix BASE-TOOL
        # Desired Pose
        T_BT_h[0:3, 0:3] = R
        # T_BT_h[0:3, 0] = x_tool[0:]
        # T_BT_h[0:3, 1] = y_tool[0:]
        # T_BT_h[0:3, 2] = z_tool[0:]
        T_BT_h[0:3, 3] = position

        # Computation of the inverse kinematics
        w_task = np.array([1, 1, 1, 1, 1, 1])
        Pose, joints_curr = InvKin.InverseKinematics(T_BT_h, joints_prev, w_task)
        joints = np.rad2deg(joints_curr)
        joints_curr =joints_curr.reshape(-1)
        joints_prev = joints_curr

        # print("\nJoints: \n")
        # print(Pose)

        # check if ptp pickup or not
        # ptp_pickup = False
        # for i in range(7):
        #     diff = joints_prev[i] - joints_curr[i]
        #     # if abs(diff) > j_max:
        #     #     ptp_pickup = True
        #     if (abs(diff)*dt) < max_speed:
        #         sendPos = True

        # if ptp_pickup:
        #     # jump too do ptp movement to pickup pose
        #     np.disp("Executing PTP movement to pickup pose.")
        #     iiwa.movePTPJointSpace(joints_curr, setup.velocityFast)

        # if sendPos:
        #jump small, start real-time control directly
        # np.disp("Executing real-time movement.")
        iiwa.sendJointsPositions(joints_curr)

        # p_old = position
        # quat_old = quat
        # joints_prev = joints_curr
        rate.sleep()

    np.disp("Not reciving others input. Moving in Home position.")
    np.disp("Have a nice day!")
    iiwa.realTime_stopDirectServoJoints()
    iiwa.movePTPJointSpace(joints_home, relVel)
    iiwa.close()

except:
    iiwa.close()

    # MATRIX TIP-CAMERA
    # T_TC = np.array([[-0.74732909654, 0.371368, 0.550981233634, 17.79907662],[-0.29891038952, -0.928486, 0.220376827192, 7.11912456],[0.593419, 0, 0.80489, -7],[0,0,0,1]])

