#!/usr/bin/env python

import numpy as np

def Rot2Quaternions(R):
    r00 = R[0,0]
    r01 = R[0,1]
    r02 = R[0,2]
    r10 = R[1,0]
    r11 = R[1,1]
    r12 = R[1,2]
    r20 = R[2,0]
    r21 = R[2,1]
    r22 = R[2,2]

    trace = r00 + r11 + r22
    epsilon = 1e-12
    if trace > epsilon:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (r21 - r12) * s
        y = (r02 - r20) * s
        z = (r10 - r01) * s
    else:
        if r00 > r11 and r00 > r22:
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

    return quat