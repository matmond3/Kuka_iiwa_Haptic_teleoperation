#!/usr/bin/env python

"""
About the script:
Computation of Inverse Kinematics for a Kuka iiwa 14 in
Python3

Created on Mon May  24 11:5p:31 2021

@author: Matteo Bermond

"""

from numpy.linalg.linalg import norm
import numpy as np
from math import cos, sin
from Rot2Quat import Rot2Quaternions

def DH_mat(DH_param):
    T = np.zeros((4, 4))

    d = DH_param[0]
    theta = DH_param[1]
    a = DH_param[2]
    alpha = DH_param[3]

    T[0, 0] = cos(theta)
    T[0, 1] = -sin(theta) * cos(alpha)
    T[0, 2] = sin(theta) * sin(alpha)
    T[0, 3] = a * cos(theta)
    T[1, 0] = sin(theta)
    T[1, 1] = cos(theta) * cos(alpha)
    T[1, 2] = -cos(theta) * sin(alpha)
    T[1, 3] = a * sin(theta)
    T[2, 0] = 0
    T[2, 1] = sin(alpha)
    T[2, 2] = cos(alpha)
    T[2, 3] = d
    T[3, 3] = 1

    return T

def getTwist(Pose_curr, Pose_des, dt):
    P_curr = Pose_curr[0:3, 3]
    P_des = Pose_des[0:3, 3]
    R_curr = Pose_curr[0:3, 0:3]
    R_des = Pose_des[0:3, 0:3]

    R_err = np.matmul(R_curr.T, R_des)

    quaternions = Rot2Quaternions(R_err)

    theta_half = np.arctan(np.sqrt(quaternions[1]**2 + quaternions[2]**2 + quaternions[3]**2)/quaternions[0])
    theta = 2*theta_half
    if not sin(theta/2) == 0:
        r = quaternions[1:]/sin(theta/2)
    else:
        r = np.array([0, 0, 1])
    r = np.matmul(R_curr, r)
    twist = np.zeros(6)
    twist = twist.reshape(-1, 1)
    twist[0:3, 0] = (P_des - P_curr) / dt
    twist[3:6, 0] = (theta / dt) * r

    return twist


class genInverseKinematics:

    def __init__(self,  type, T_init=np.identity(4), T_tool=np.identity(4)) :
        self.T_tool = T_tool
        if type == 'LBR14R820':
            x = np.array([[0.36, 3.1415926, 0, 1.5707963], [0, 3.1415926, 0, 1.5707963], [0.42, 0, 0, 1.5707963],
                      [0, 3.1415926, 0, 1.5707963], [0.4, 0, 0, 1.5707963], [0, 3.1415926, 0, 1.5707963],
                      [0.1257, 0, 0, 0]])

        elif type == 'LBR7R800':
            x = np.array([[0.34, 3.1415926, 0, 1.5707963], [0, 3.1415926, 0, 1.5707963], [0.4, 0, 0, 1.5707963],
                          [0, 3.1415926, 0, 1.5707963], [0.4, 0, 0, 1.5707963], [0, 3.1415926, 0, 1.5707963],
                          [0.1257, 0, 0, 0]])

        else:
            np.disp("The specified Robot name shall be either 'LBR14R820' or 'LBR7R800'.")

        self.DH_table = np.asmatrix(x)
        self.n = 7  # number of joitns
        self.n_ctrl = 7
        self.jtype = 'rrrrrrr'  # joint type --> 'r'=revolute and 'p'=prismatic
        self.T_init = np.identity(4)

        self.damping = 1e-3
        self.Err = 1e-6
        self.dErr = 1e-10

        self.q_lims_max = np.array(np.deg2rad([[170], [120], [170], [120], [170], [120], [170]]))
        self.q_lims_min = -self.q_lims_max
        self.T_init = T_init

    # Definition of the joints pose, q is a vector (1,7)
    # GETPOSE is a function to compute the pose of the end effector T

    def getPose(self,q):
        Pose = np.copy(self.T_init)
        DH_table = np.copy(self.DH_table)

        for i in range(self.n):
            DH_params = DH_table[i,:]

            if self.jtype[i] == 'r':
                DH_params[1] = DH_params[1] + q[i]
            elif self.jtype[i] == 'p':
                DH_params[0] = DH_params[0] + q[i]

            T_i = DH_mat(DH_params)
            Pose = Pose.dot(T_i)
        Pose = np.matmul(Pose, self.T_tool)
        return Pose

    # COMPUTATION OF EE JACOBIAN
    def getJacobian(self, q):

        T_ee = self.getPose(q)
        r_ee = T_ee[0:3,3]
        Pose = np.copy(self.T_init)
        J = np.zeros((6, self.n))
        J_tot = np.zeros((6, self.n))
        DH_tab = np.copy(self.DH_table)
        for i in range(self.n):
            z_0 = Pose[0:3, 2]

            DH_params = DH_tab[i, :]

            if self.jtype[i] == 'r':
                DH_params[1] = DH_params[1] + q[i]
            elif self.jtype[i] == 'p':
                DH_params[0] = DH_params[0] + q[i]

            r_i = Pose[0:3, 3]
            r = r_ee - r_i

            if self.jtype[i] == 'r':

                J_tot[0:3, i] = np.cross(z_0, r)
                J_tot[3:6, i] = z_0

            elif self.jtype[i] == 'p':

                J_tot[0:3, i] = z_0

            T_i = DH_mat(DH_params)
            Pose = Pose.dot(T_i)

        J = J_tot

        return J

    ## IK PARAM DEFINITION

    ##Pose_des
    # Pose_des is a 4-by-4 matrix have to set it as Pose_des=eye(4),
    # then using Pose_des(1:3,4)= [X,Y,Z] I can set the coordinates
    # desired for the end effector.

    ## q
    # is the starting joints pose!

    ## w_task
    # is a 6-by-1 vector which define the kind of task we want to
    # perform, if just focused on the cartesian coordinates or on the
    # pose too:
    # w_task=[1,1,1,0,0,0] works just on the x,y,z poistion
    # w_task=[1,1,1,1,1,1] works on x,y,z position and
    # wx,wy,wz orientations

    def InverseKinematics(self, Pose_des, q_row, w_task):

        q = q_row
        dt = 0.9
        err = 2* self.Err
        iter = 0
        Iter = 1500
        err_old = err

        W_task = np.diag(w_task)

        while (err > self.Err and iter < Iter):

            Pose = self.getPose(q)
            J = self.getJacobian(q)

            twist = getTwist(Pose, Pose_des, dt)
            twist = np.matmul(W_task, twist)
            J = np.matmul(W_task, J)

            J_pinv = np.linalg.pinv(J)

            err = np.linalg.norm(twist)
            derr = err - err_old
            q_ave = (self.q_lims_max + self. q_lims_min)/2
            q = q.reshape(-1, 1)
            qd_null = q_ave - q
            N = np.identity(7) - np.matmul(J_pinv, J)  # Proiettore del NULL space
            dq = np.matmul(J_pinv, twist) + np.matmul(N, qd_null)

            # dq = np.matmul(J_pinv, twist)
            q = q + dt * dq
            q = np.arctan2(np.sin(q), np.cos(q))
            iter = iter + 1

            if (abs(derr) < self.dErr):
                break

        q_out = np.zeros(range(self.n))

        q_out = np.minimum(q, self.q_lims_max)
        q_out = np.maximum(q_out, self.q_lims_min)

        Pose = self.getPose(q_out)

        return Pose, q_out