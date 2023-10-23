#!/usr/bin/env python
import math
import numpy as np
from math import  cos,sin, pi
from scipy.spatial.transform import Rotation as sc_rot

"""
    Define the Rotation matrix derived by a RPY angles set.
    The input is a vector 3-by-1
    The output is a matrix 3-by-3
"""

def RPYtoRotation(rpy):

    R=sc_rot.from_euler('xyz', rpy, degrees=False)
    R=R.as_dcm()

    """
    theta_x = rpy[0]
    theta_y = rpy[1]
    theta_z = rpy[2]

    # In order to avoid singularities
    if theta_y == pi/2:
        theta_y = np.deg2rad(89)

    r00 = cos(theta_x) * cos(theta_z)
    r01 = -cos(theta_y) * sin(theta_z)
    r02 = sin(theta_y)
    r10 = sin(theta_x)*sin(theta_y)*cos(theta_z) + cos(theta_x)*sin(theta_z)
    r11 = -sin(theta_x)*sin(theta_y)*sin(theta_z) + cos(theta_x)*cos(theta_z)
    r12 = -sin(theta_x) * cos(theta_y)
    r20 = -cos(theta_x)*sin(theta_y)*cos(theta_z) + sin(theta_x)*sin(theta_z)
    r21 = cos(theta_x)*sin(theta_y)*sin(theta_z) + sin(theta_x)*cos(theta_z)
    r22 = cos(theta_x) * cos(theta_y)

    R = np.array([[r00, r01, r02], [r10,r11,r12], [r20, r21, r22]])
    """

    return R