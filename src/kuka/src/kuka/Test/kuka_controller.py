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
import sys
import yaml
import numpy as np
import rospy
import threading

sys.path.insert(1, '/home/matteo/catkin_ws/src/Kuka/src/scripts/python_client')
from python_client.QuaternionToRotation import quaternion_rotation_matrix
from python_client.RPYtoRotation import RPYtoRotation
from python_client.genInverseKinematics import genInverseKinematics
from geometry_msgs.msg import PoseStamped
from python_client.setupROS import setupROS
from python_client.iiwaPy3 import iiwaPy3

# LOADING OF WORKSPACE PARAMETERS FROM YAML FILE
with open(r'/home/matteo/catkin_ws/src/kuka/kuka_control/config/kuka_params_rt.yaml') as file:
    info = yaml.full_load(file)

    # "params" are loaded as a dictionary so to access the informations stored in it you have to use the
    # following syntax: params['key_word'].
    # e.g. params['robot'] return the following string 'LBR14R820'

kukaType = info['robot']
# Connect to the robot
ip = '192.170.10.106'
# ip='localhost'
iiwa = iiwaPy3(ip)
setup = setupROS(info)
InvKin = genInverseKinematics(kukaType, np.identity(4))


class Callback:
    def __init__(self):
        self.pose = PoseStamped()
        self.sub = rospy.Subscriber("phantom/pose", PoseStamped, self.callback, queue_size=1000000)

    def callback(self, msg):
        self.pose = msg

    def get_msg(self):
        """
        Returns the newest Pose data
        """
        return self.pose


if __name__ == '__main__':

    try:
        relVel = [0.1]
        jointsHome, T_BT_h, T_BF_h = setup.setupWorkspace()
        iiwa.movePTPJointSpace(jointsHome, relVel)

        # Creation of ROS Node
        rospy.init_node('Haptic_EE_listener')
        read_pose = Callback()
        # timer = threading.Timer(5,timeout)
        # timer.start()
        rospy.sleep(2)
        rate = rospy.Rate(1000)

        # Counter to exit the code
        n = 0

        # # Save the previous position of the Haptic_device
        data_old = read_pose.get_msg()
        quat_old = np.array([data_old.pose.orientation.w, data_old.pose.orientation.x, data_old.pose.orientation.y,
                             data_old.pose.orientation.z])
        np.disp('QUAT_OLD = ' + str(quat_old))
        p_old = np.array([data_old.pose.position.x, data_old.pose.position.y, data_old.pose.position.z])
        #
        while not rospy.is_shutdown():

            # Get the actual pose of the Haptic device
            data = read_pose.get_msg()
            quat = np.array(
                [data.pose.orientation.w, data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z])
            p = np.array([data.pose.position.x, data.pose.position.y, data.pose.position.z])

            np.disp('')
            np.disp('QUAT = ' + str(quat))
            np.disp('P = ' + str(p))
            np.disp('p_old = ' + str(p_old))
            np.disp('quat_old = ' + str(quat_old))

            # computation of DELTA_position
            desired_pose = p - p_old

            np.disp('p_des = ' + str(desired_pose))
            np.disp('')
            np.disp('')

            # computattion of DELTA_Rotation

            R_old = quaternion_rotation_matrix(quat_old)
            R_new = quaternion_rotation_matrix(quat)
            Delta_R = np.matmul(R_old.T, R_new)

            # Computation of the rotation matrix to send to KUKA IIWA
            # gives position and quaternions of the EEF
            T_EE_KUKA = iiwa.getEEFPos()
            rpy_EEF_old = T_EE_KUKA[3:]
            R_kuka = RPYtoRotation(rpy_EEF_old)
            R = np.matmul(R_kuka, Delta_R)

            P_des = np.identity(4)
            P_des[0:3, 0:3] = R
            P_des[0, 3] = p[0]
            P_des[1, 3] = p[1]
            P_des[2, 3] = p[2]

            q_row = iiwa.getJointsPos()

            w_task = np.array([1, 1, 1, 0, 0, 0])

            # Pose, jPos = InvKin.InverseKinematics(P_des, q, w_task)
            #
            # relVel = [0.1]
            # jPos = jPos.reshape(-1)
            # #iiwa.movePTPJointSpace(jPos, relVel)
            # EE_Pos = iiwa.getEEFPos()
            # np.disp(str(EE_Pos))
            #
            p_old = p
            quat_old = quat
            n = n + 1

            if n == 10000:
                iiwa.close()
            rate.sleep()

    except:
        print('an error happened')
        iiwa.close()
