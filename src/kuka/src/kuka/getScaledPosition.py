#!/usr/bin/env python

import numpy as np

def getScaledPosition(position, ws_center,scaling_factor):
    # GETSCALEDPOSITION Scales tool position around workspace center.

    # uniform scaling matrix
    scaling_matrix = np.identity(3) * scaling_factor

    # make workspace center new origin
    position = position - ws_center

    # scaling around workspace center
    position = np.matmul(scaling_matrix, position)

    # translate back to KUKA origin
    position = position + ws_center

    return position