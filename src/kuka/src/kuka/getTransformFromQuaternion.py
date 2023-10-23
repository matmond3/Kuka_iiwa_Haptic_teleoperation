#!/usr/bin/env python

import numpy as np

from QuaternionToRotation import quaternion_rotation_matrix


def getTransformFromQuaternion(position, quaternions):
    # GETTRANSFORMFROMQUATERNION Returns transformation matrix.
    # calculates rotation matrix from quaternions and then assembles
    # the full 4 by 4 transformation including position.

    x_pos = position[0]
    y_pos = position[1]
    z_pos = position[2]

    R = quaternion_rotation_matrix(quaternions)

    T = np.identity(4)
    T[0:3,0:3] = R
    T[0:3, 3] = np.array([x_pos, y_pos, z_pos])

    return T