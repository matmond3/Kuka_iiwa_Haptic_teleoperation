#!/usr/bin/env python

import numpy as np
from math import sin, cos, pi

def Rpy2Quaternions(rpy):
    theta_x = rpy[0]
    theta_y = rpy[1]
    theta_z = rpy[2]

    # In order to avoid singularities
    if theta_y == pi / 2:
        theta_y = np.deg2rad(89)

    r00 = cos(theta_x) * cos(theta_z)
    r01 = -cos(theta_y) * sin(theta_z)
    r02 = sin(theta_y)
    r10 = sin(theta_x) * sin(theta_y) * cos(theta_z) + cos(theta_x) * sin(theta_z)
    r11 = -sin(theta_x) * sin(theta_y) * sin(theta_z) + cos(theta_x) * cos(theta_z)
    r12 = -sin(theta_x) * cos(theta_y)
    r20 = -cos(theta_x) * sin(theta_y) * cos(theta_z) + sin(theta_x) * sin(theta_z)
    r21 = cos(theta_x) * sin(theta_y) * sin(theta_z) + sin(theta_x) * cos(theta_z)
    r22 = cos(theta_x) * cos(theta_y)

    R = np.array([[r00, r01, r02], [r10, r11, r12], [r20, r21, r22]])

    trace = r00 + r11 + r22
    epsilon = 1e-12
    if trace > epsilon:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (r21 - r12) * s
        y = (r02 - r20) * s
        z = (r10 - r01) * s
    else:
        if  r00 > r11 and r00 > r22:
            s = 2.0 * np.sqrt(1.0 + r00 - r11 - r22)
            w = (r21 - r12) / s
            x = 0.25 * s
            y = (r01 + r10) / s
            z = (r02 + r20) / s
        elif (r11 > r22):
            s = 2.0 * np.sqrt(1.0 + r11 - r00 - r22)
            w = (r02 - r20) / s
            x = (r01 + r10) / s
            y = 0.25 * s
            z = (r12 + r21) / s
        else:
            s = 2.0 * np.sqrt(1.0 + (r22 - r00 - r11))
            w = (r10 - r01) / s
            x = (r02 + r20) / s
            y = (r12 + r21) / s
            z = 0.25 * s

    quat = np.array([w, x, y, z])

    return  quat