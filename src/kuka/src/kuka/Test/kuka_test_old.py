#!/usr/bin/env python
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
sys.path.insert(1, '/home/matteo/catkin_ws/src/Kuka/src/scripts/python_client')
import yaml
import time
import numpy as np
import rospy

from python_client.getConstrainedPosition import getConstrainedPosition
from python_client.getScaledPosition import getScaledPosition
from python_client.RPY2Quaternions import Rpy2Quaternions

from python_client.genInverseKinematics import genInverseKinematics
from python_client.setupROS import setupROS
from python_client.iiwaPy3 import iiwaPy3
from geometry_msgs.msg import Pose
from python_client.QuaternionToRotation import quaternion_rotation_matrix

# LOADING OF WORKSPACE PARAMETERS FROM YAML FILE
with open('/home/matteo/catkin_ws/src/kuka/src/config/kuka_params_rt.yaml') as file:
    info = yaml.full_load(file)
    print(info)
    # "params" are loaded as a dictionary so to access the informations stored in it you have to use the
    # following syntax: params['key_word'].
    # e.g. params['robot'] return the following string 'LBR14R820'

kukaType = info['robot']
# Connect to the robot
ip = '192.170.10.106'
iiwa = iiwaPy3(ip)
setup = setupROS(info)
InvKin = genInverseKinematics(kukaType, np.identity(4))
scaling_factor = 1
j_max = np.deg2rad(2) #setting of the maximum allowed angle for the first movement

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

if __name__ == '__main__':
	try:

	    # Creation of ROS Node - ROS Publisher
	    rospy.init_node('Haptic_EE_listener')
	    read_pose = Callback()
	    pub = rospy.Publisher('kuka_position', Pose, queue_size=1000)
	    rospy.sleep(2)
	    rate = rospy.Rate(10)

	    # Move to home position
	    relVel = [0.1]
	    jointsHome, T_BT_h, T_BF_h, ws_center = setup.setupWorkspace()
	    iiwa.movePTPJointSpace(jointsHome, relVel)
	    joints_prev = jointsHome

	    EEF_homePose = iiwa.getEEFPos()
	    np.disp(EEF_homePose)
	    np.disp("\nHome Pose:\n")

	    # Are bot Robots in Home position?
	    data_home = read_pose.get_msg()
	    p_home = np.array([data_home.position.x, data_home.position.y, data_home.position.z])
	    # The OMNI pose is send as a Pose msg; so I directly obtain the position and the orientation
	    # through quaternion of the EEF of the OMNI
	    data_home = read_pose.get_msg()
	    quat_old = np.array([data_home.orientation.w, data_home.orientation.x, data_home.orientation.y,
		                 data_home.orientation.z])


	    # rpy = np.array([EEF_homePose[3], EEF_homePose[4], EEF_homePose[5]])
	    # quat = Rpy2Quaternions(rpy)
	    p_old = p_home
	    R_HK = np.array([[-1, 0, 0], [0, -1, 0], [0, 0, 1]])
	    p_old = np.matmul(R_HK, p_old)

	    value = input("Ready for RT motion?\n")

	    while not rospy.is_shutdown():
		actual_kuka_pose = iiwa.getEEFPos()
		actual_kuka_position = np.array([actual_kuka_pose[0]/1000, actual_kuka_pose[1]/1000, actual_kuka_pose[2]/1000])
		# Get the actual pose of the Haptic device
		data = read_pose.get_msg()
		quat = np.array([data.orientation.w, data.orientation.x, data.orientation.y, data.orientation.z])
		# POSITION FROM HAPTIC DEVICE transfromed from METERS to MILLIMETERS
		p = 1000 * np.array([data.position.x, data.position.y, data.position.z])
		p = np.matmul(R_HK, p)

		# computation of DELTA_position
		position = p - p_old
		print("\nDelta movement\n:")
		print(position)
		# Scaled around Workspace center
		position = getScaledPosition(position, ws_center, scaling_factor)
		print("\nScale \n")
		print(position)

		position = actual_kuka_position + position
		print("\nMovement:\n")
		print(position)

		# computattion of DELTA_Rotation

		R_old = quaternion_rotation_matrix(quat_old)
		R_new = quaternion_rotation_matrix(quat)
		R = np.matmul(R_old.T, R_new)
		x_tool = np.array([0, 0, -1])
		z_tool = setup.center_line
		z_tool_norm = z_tool / np.linalg.norm(z_tool)
		y_tool = np.cross(z_tool_norm, x_tool)

		p_old = p
		quat_old = quat

		# Enforce workspace boundary constrained if necessary
		position, error, inside = getConstrainedPosition(position, setup)

		print("\nConstrained position:\n")
		print(position)

		kukaPos = Pose()
		kukaPos.position.x = position[0] - actual_kuka_position[0]
		kukaPos.position.y = position[1] - actual_kuka_position[1]
		kukaPos.position.z = position[2] - actual_kuka_position[2]
		pub.publish(kukaPos)

		if error:
		    np.disp("Shutting down the application.")
		    break

		# Construction of the 4-by-4 homogeneous matrix BASE-TOOL
		# Desired Pose
		# T_BT_h[0:3, 0:3] = R
		T_BT_h[0:3, 0] = x_tool[0:]
		T_BT_h[0:3, 1] = y_tool[0:]
		T_BT_h[0:3, 2] = z_tool_norm[0:]
		T_BT_h[0:3, 3] = position

		# Computation of the inverse kinematics
		w_task = np.array([1, 1, 1, 1, 1, 1])
		Pose, joints_curr = InvKin.InverseKinematics(T_BT_h, joints_prev, w_task)
		joints = np.rad2deg(joints_curr)
		joints_curr =joints_curr.reshape(-1)
		joints_prev = joints_curr

		print("\nJoints: \n")
		print(joints)

		# check if ptp pickup or not
		ptp_pickup = False
		for i in range(7):
		    diff = jointsHome[i] - joints_curr[i]
		    if abs(diff) > j_max:
		        ptp_pickup = True

		#         if ptp_pickup:
		#     # jump too do ptp movement to pickup pose
		#     np.disp("Executing PTP movement to pickup pose.")
		#     iiwa.movePTPJointSpace(joints_curr, setup.velocityFast)
		#
		# else:
		#     # jump small, start real-time control directly
		#     np.disp("Executing real-time movement.")
		#     iiwa.realTime_startDirectServoJoints()
		#     iiwa.sendJointsPosition(joints_curr)


		rate.sleep()



	    np.disp("Not reciving others input. Moving in Home position.")
	    np.disp("Have a nice day!")
	    iiwa.movePTPJointSpace(jointsHome, relVel)
	    iiwa.realTime_stopDirectServoJoints()
	    iiwa.close()


	except:
	    iiwa.close()

