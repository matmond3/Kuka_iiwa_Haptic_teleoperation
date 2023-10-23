#!/usr/bin/env python

import math
import numpy as np


def checkSolutionValidity(joints_curr, joints_prev, setup):
    # CHECKSOLUTIONVALIDITY Checks if inverse kinematics solution is valid.

    for i in range(len(joints_curr)):
        if math.isnan(joints_curr[i]):
            np.disp("No IK solution found. Staying in previous pose.")
            joints_curr = joints_prev
            return joints_curr

    in_bounds, joints_curr = setup.checkJointLimits(joints_curr)
    if not in_bounds:
        np.disp("Joint limit violated. Staying in previous pose.")

    return joints_curr

