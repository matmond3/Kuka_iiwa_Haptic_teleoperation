#!/usr/bin/env python

import numpy as np
from math import cos, sin


def getConstrainedPosition(position, setup):
    # Get constrained Position ENFORCES workspace boundary constraints

    error = False
    inside = True
    position_sendHaptic = np.array([0.0, 0.0, 0.0])

    sphere_origin = np.array([0, 0, setup.DH_table[0, 0]])

    # 1. FEASIBILITY CHECK
    if position.dot(setup.center_line) < 0:
        #np.disp("Error. Desired position not in positive half of robot.")
        error = True
        inside = False
        return
    if position[2] < 0:
        #np.disp("Error. Desired position lies below KUKA XY plane.")
        error = True
        inside = False

    # 2. ENFORCE OPENING ANGLES

    OA = np.copy(np.deg2rad(setup.opening_angle))
    R_z_r = np.array([[cos(-OA / 2), - sin(-OA / 2), 0],
                      [sin(-OA / 2), cos(-OA / 2), 0], [0, 0, 1]])
    R_z_l = np.array([[cos(OA / 2), - sin(OA / 2), 0],
                      [sin(OA / 2), cos(OA / 2), 0], [0, 0, 1]])

    plane_r_dir = np.matmul(np.copy(setup.center_line),R_z_r.T)
    plane_l_dir = np.matmul(np.copy(setup.center_line), R_z_l.T)
    plane_r_dir = plane_r_dir / np.linalg.norm(plane_r_dir)
    plane_l_dir = plane_l_dir / np.linalg.norm(plane_l_dir)

    plane_r_normal = np.cross(plane_r_dir, [0, 0, 1])
    plane_l_normal = -np.cross(plane_l_dir, [0, 0, 1])

    if position.dot(plane_r_normal) > 0:
        np.disp("Enforcing right opening angle constraint!")
        pos_rif_r = np.array([0, position[1], position[0]])
        # vector projection
        a1 = position[0:2].dot(plane_r_dir[0:2]) * plane_r_dir[0:2]
        a2 = position[0:2] - a1
        position[0: 2] = position[0: 2] - a2
        position_updated_r = np.array([0, position[1], position[0]])
        position_sendHaptic = pos_rif_r - position_updated_r
        inside = False

    elif np.dot(position, plane_l_normal) > 0:
        np.disp("Enforcing left opening angle constraint,")
        # vector projection
        pos_rif_l = np.array([0, position[1], position[0]])
        a1 = position[0:2].dot(plane_l_dir[0: 2]) * plane_l_dir[0: 2]
        a2 = position[0:2] - a1
        position[0: 2] = position[0: 2] - a2
        position_updated_l = np.array([0, position[1], position[0]])
        position_sendHaptic = pos_rif_l - position_updated_l

        inside = False

    # 3. ENFORCE Z PLANE CONSTRAINTS
    if position[2] < (setup.zLowerLimit + setup.tool_length):
        position_sendHaptic[0] = (setup.zLowerLimit + setup.tool_length) - position[2]
        position[2] = (setup.zLowerLimit + setup.tool_length)
        #np.disp("Enforcing lower z limit.")
        inside = False

    elif position[2] > (setup.zUpperLimit + setup.tool_length):
        position_sendHaptic[0] = (setup.zUpperLimit + setup.tool_length) - position[2]
        position[2] = (setup.zUpperLimit + setup.tool_length)
        #np.disp("Enforcing upper z limit.")
        inside = False

    # 4. ENFORCE SPHERE CONSTRAINTS

    # translate vector from sphere origin to KUKA origin
    position = position - sphere_origin
    z = position[2]


    # get euclidean distance between origin and desired position
    dist = np.linalg.norm(position)
    arr = np.array([0, 0, z])

    if dist < setup.inner_sphere:
        position_sendHaptic[2] = setup.inner_sphere - dist
        # find desired offset from z axis with pythagoras
        offset = np.sqrt(np.copy(setup.inner_sphere) ** 2 - z ** 2)
        # find unit vector in x,y plane and translate accordingly
        direction = position - [0, 0, z]
        unit_vector = direction / np.linalg.norm(direction)
        position = (offset * unit_vector) + arr

        #np.disp("Enforcing inner sphere constraint.")
        inside = False

    elif dist > setup.outer_sphere:
        position_sendHaptic[2] = setup.outer_sphere - dist
        # find desired offset from z axis with pythagoras
        offset = np.sqrt(np.copy(setup.outer_sphere) ** 2 - z ** 2)
        # find unit vector in x,y plane and translate accordingly
        direction = position - [0, 0, z]
        unit_vector = direction / np.linalg.norm(direction)
        position = (offset * unit_vector) + arr

        #np.disp("Enforcing outer sphere constraint.")
        inside = False

    position = position + sphere_origin

    return position, error, inside, position_sendHaptic
