#! /usr/bin/env python3

import sys
import yaml
import time
import numpy as np
import rospy
from getConstrainedPosition import getConstrainedPosition
from getScaledPosition import getScaledPosition
from RPY2Quaternions import Rpy2Quaternions
from Rot2Quat import Rot2Quaternions

from genInverseKinematics import genInverseKinematics
from setupROS import setupROS
from geometry_msgs.msg import Pose
from QuaternionToRotation import quaternion_rotation_matrix

# sys.path.append('/home/matteo/catkin_ws/src/kuka/src/kuka/getConstrainedPosition.py')
with open('/home/matteo/catkin_ws/src/kuka/src/config/kuka_params_rt.yaml') as file:
    info = yaml.full_load(file)

class Callback:
    def __init__(self):
        self.pose = Pose()
        self.sub = rospy.Subscriber("Haptic_Dev/pose", Pose, self.callback, queue_size = 10000)

    def callback(self, msg):
        self.pose = msg

    def get_msg(self):
        """
        Returns the newest Pose data
        """
        return self.pose

kukaType = info['robot']
# Connect to the robot
setup = setupROS(info)
InvKin = genInverseKinematics(kukaType, np.identity(4))
scaling_factor = 0.8

if __name__ == '__main__':

    # Definition of the variable to send to the Haptic device for the Haptic feedback creation
    kukaPos = Pose()

    # Creation of ROS Node
    rospy.init_node('Tranformation_Node')
    read_pose = Callback()
    pub = rospy.Publisher('Pose4Kuka', Pose, queue_size=10000)
    rospy.sleep(2)
    rate = rospy.Rate(1000)

    relVel = [0.1]
    w_task = np.array([1, 1, 1, 1, 1, 1])
    jointsHome, T_BT_h, T_BF_h, ws_center = setup.setupWorkspace()
    PoseHome, joints_home = InvKin.InverseKinematics(T_BF_h, jointsHome, w_task)

    kuka_homePosition = np.array([PoseHome[0,3], PoseHome[1,3], PoseHome[2,3]])

    # Save the previous position of the Haptic_device
    data_old = read_pose.get_msg()
    quat_old = np.array([data_old.orientation.w, data_old.orientation.x, data_old.orientation.y, data_old.orientation.z])
    p_old = np.array([data_old.position.x, data_old.position.y, data_old.position.z])

    # The OMNI pose is send as a Pose msg; so I directly obtain the position and the orientation
    # through quaternion of the EEF of the OMNI

    z_tool = np.array([0, 0, -1])
    x_tool = - setup.center_line
    x_tool_norm = x_tool / np.linalg.norm(x_tool)
    y_tool = np.cross(z_tool, x_tool_norm)

    T = np.identity(4)
    T[0:3, 0] = x_tool_norm
    T[0:3, 1] = y_tool
    T[0:3, 2] = z_tool
    T[0:3, 3] = kuka_homePosition
    T[3, 3] = 1

    # rpy = np.array([EEF_homePose[3], EEF_homePose[4], EEF_homePose[5]])
    # quat = Rpy2Quaternions(rpy)
    R_centerline = setup.getRotationFromBasis(x_tool, y_tool, z_tool)
    # R_45 = np.array([[(np.sqrt(2)/2), -(np.sqrt(2)/2), 0], [(np.sqrt(2)/2), (np.sqrt(2)/2), 0], [0, 0, 1]])
    R_Y = np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]])
    R_HK = np.array([[0, 0, 1], [0, -1, 0], [1, 0, 0]])
    R_HK_pos = np.matmul(R_HK, R_centerline)

    while not rospy.is_shutdown():
        # POSITION
        # Get the actual pose of the Haptic device
        data = read_pose.get_msg()
        quat = np.array([data.orientation.w, data.orientation.x, data.orientation.y, data.orientation.z])
        # POSITION FROM HAPTIC DEVICE transformed from METERS to MILLIMETERS
        pos_haptic_or = np.array([data.position.x, data.position.y, data.position.z])

        pos_haptic_scaled = scaling_factor * pos_haptic_or
        pos_haptic = np.matmul(pos_haptic_scaled, R_HK_pos)
        p_temp = np.array([pos_haptic[0], pos_haptic[1], pos_haptic[2], 1])

        p = np.matmul(T, p_temp)
        position = np.array([p[0], p[1], p[2]])

        # # Scaled around Workspace center
        # position = getScaledPosition(position, ws_center, scaling_factor)
        # print("\nScale \n")
        # print(position)
        # position_sendHaptic = np.array([position[0], position[1], position[2], 1])
        # pos_kuka_temp = np.matmul(position_sendHaptic, np.linalg.inv(T))
        # pos_kuka = np.array([pos_kuka_temp[0], pos_kuka_temp[1], pos_kuka_temp[2]])

        # Enforce workspace boundary constrained if necessary
        position, error, inside, position_sendHaptic = getConstrainedPosition(position, setup)

        # ORIENTATION
        R = quaternion_rotation_matrix(quat)
        R = np.matmul(R, R_centerline)
        quat = Rot2Quaternions(R)


        pos_kuka = position
        kukaPos.position.x = pos_kuka[0]
        kukaPos.position.y = pos_kuka[1]
        kukaPos.position.z = pos_kuka[2]
        kukaPos.orientation.w = quat[0]
        kukaPos.orientation.x = quat[1]
        kukaPos.orientation.y = quat[2]
        kukaPos.orientation.z = quat[3]
        pub.publish(kukaPos)

        rate.sleep()