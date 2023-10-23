#!/usr/bin/env python
import numpy as np
from math import cos,sin,pi

class setupROS:

    def __init__(self, info):
        x = np.zeros((7 ,4))
        self.name = str(info['robot'])
        self.flange = str(info['flange'])
        self.ip = str(info['ip'])
        self.center_line = np.array(info['center_line'])
        self.opening_angle = float(info['opening_angle'])
        self.zLowerLimit = float(info['z_lower_limit'])
        self.zUpperLimit = float(info['z_upper_limit'])
        self.inner_sphere = float(info['inner_sphere_limit'])
        self.outer_sphere = float(info['outer_sphere_limit'])
        self.tool_length = float(info['tool_length'])
        self.tool_thickness = float(info['tool_thickness'])
        self.velocitySlow = [float(info['velocity_ptp_slow'])]
        self.velocityFast = [float(info['velocity_ptp_fast'])]
        self.timeout = int(info['time_out'])
        self.home_pose = np.deg2rad(np.array(info['home_pos']))
        self.useHomePos = bool(info['use_home_pos'])

        if self.name == "LBR7R800":
            x = np.array([[0.34, 3.1415926, 0, 1.5707963], [0, 3.1415926, 0, 1.5707963], [0.4, 0, 0, 1.5707963],
                          [0, 3.1415926, 0, 1.5707963], [0.4, 0, 0, 1.5707963], [0, 3.1415926, 0, 1.5707963],
                          [0.1257, 0, 0, 0]])

        elif self.name == "LBR14R820":
            x = np.array([[0.36, 3.1415926, 0, 1.5707963], [0, 3.1415926, 0, 1.5707963], [0.42, 0, 0, 1.5707963],
                          [0, 3.1415926, 0, 1.5707963], [0.4, 0, 0, 1.5707963], [0, 3.1415926, 0, 1.5707963],
                          [0.1257, 0, 0, 0]])
        self.DH_table = x
        # Transformation from Flange to tool tip
        self.T_FT = np.identity(4)
        self.T_FT[0, 3] = self.tool_length
        self.T_FT[2, 3] = self.tool_thickness
        # Joints Limits
        self.q_lims_max = np.deg2rad(np.array([[150], [120], [150], [120], [150], [120], [170]]))
        self.q_lims_min = -self.q_lims_max

    def setupWorkspace(self):
        self.checkWorkspace()
        joints_home = np.zeros(7)
        T_BT = np.identity(4)
        T_BF = np.identity(4)
        if self.useHomePos:
            # controll if the desired position leads inside the Work Space
            ws_center, ws_verify = self.checkWSValidity()

            if ws_verify == "true":
                # used specified home pose
                joints_home = np.copy(self.home_pose)

            else:
                np.disp(' ')
                np.disp("***** ERROR: The desired configuration leads to an EEF pose laying outside the desired work space.")
                np.disp(' ')
                joints_home = np.copy(self.home_pose)

        elif not self.useHomePos:
            # Computation of the workspace center
            ws_center = self.getWorkspaceCenter()
            # np.disp(' ')
            # np.disp('The work space center is: ')
            # np.disp(ws_center)
            # Home pose computation
            joints_home, T_BT, T_BF = self.getJointsHome(ws_center)

        return joints_home, T_BT, T_BF, ws_center

    def getWorkspaceCenter(self):

        # Calculate WS center as midpoint of the workspace.
        sphere_origin = np.array([0, 0, np.copy(self.DH_table[0,0])])

        # Center lies above the center_line and between the upper and lower z limit
        z = (np.copy(self.zUpperLimit) + np.copy(self.zLowerLimit)) / 2

        mean_sphere_radius = (np.copy(self.inner_sphere) + np.copy(self.outer_sphere)) / 2
        alpha = np.arcsin((z - sphere_origin[2]) / mean_sphere_radius)

        # offset of center from KUKA z axis
        z_offset = cos(alpha) * mean_sphere_radius

        center_x = np.array([z_offset, 0, z])
        unit_center_line = np.copy(self.center_line) / np.linalg.norm(np.copy(self.center_line))

        theta = np.arctan(self.center_line[1]/self.center_line[0])

        R = np.array([[cos(theta), sin(theta), 0], [-sin(theta), cos(theta), 0], [0, 0, 1]])


        # normalize center line vector for scaling
        direction = np.copy(self.center_line) / np.linalg.norm(np.copy(self.center_line))

        # scale direction vector by z offset to obtain x,y coordinates of center
        # center = direction * z_offset
        # center[2] = z

        center = np.matmul(center_x, R)

        return center

    def checkWorkspace(self):
        # check if GIVEN WORKSPACE PARAMETERS ARE VALID
        # Gives an error when any condition is violated.

        # Workspace parameter limits
        max_opening_angle = 180
        min_opening_angle = np.deg2rad(0)
        min_z = 0.1
        max_z = 1.14
        min_inner_sphere = 0.4
        max_outer_sphere = 0.8
        sphere_origin = np.array([0, 0, (np.copy(self.DH_table[0,0]))])

        if self.center_line[2] != 0:
            np.disp("[ERROR]: Z component of center line is not zero. Center line MUST lie in xy-plane")
        elif self.opening_angle <= min_opening_angle:
            np.disp("[ERROR]: Workspace opening angle MUST be larger than" + str(
                np.rad2deg(min_opening_angle)) + " degree.")
        elif self.opening_angle >= max_opening_angle:
            np.disp("[ERROR]: Workspace opening angle MUST be smaller than " + str(
                np.rad2deg(max_opening_angle)) + " degree.")
        elif self.zLowerLimit < min_z:
            np.disp("[ERROR]: Workspace lower z limit must be larger than " + str(min_z) + " meters.")
        elif self.zUpperLimit >= max_z:
            np.disp("[ERROR]: Workspace lower z limit must be smaller than " + str(max_z) + " meters.")
        elif self.zUpperLimit <= self.zLowerLimit:
            np.disp("[ERROR]: Workspace upper z limit MUST be larger than lower z limit.")
        elif self.zUpperLimit > (sphere_origin[2] + self.outer_sphere):
            np.disp("[ERROR]: Workspace upper z limit MUST NOT exceed the joint space. The maximum allowed value for upper z limit is" + str(self.outer_sphere + sphere_origin[2]) + "meters.")
        elif self.inner_sphere <= min_inner_sphere:
            np.disp("[ERROR]: Workspace inner sphere limit, must be larger than" + str(min_inner_sphere) + " meters.")
        elif self.outer_sphere >= max_outer_sphere:
            np.disp("[ERROR]: Workspace outer sphere limit, must be smaller than" + str(max_outer_sphere) + " meters.")
        elif self.outer_sphere <= self.inner_sphere:
            np.disp("[ERROR]: Outer sphere limit must be larger than inner sphere limit.")
        elif self.tool_length < 0 or self.tool_thickness < 0:
            np.disp("[ERROR]: Tool dimensions must not be negative.")

        return

    def getJointsHome(self, ws_center):
        # Calculates home position of KUKA in joint space!

        #   Definition of home position:
        #   1. Tool position must coincide with workspace center
        #   2. Tool z axis bust point through center_line direction and align with ws_center
        #   3. Tool x axis must point downwards
        #   4. When viewed from above KUKA links should all lie above center line

        #   Approach for simplified inverse kinematics for KUKA Home Pose
        #   A. Calculate FLANGE POSE such that tool pose satisfies the constraints
        #   B. Joints 3, 5 and 7 are fixed (i.e. 0)
        #   C. Calculate JOINT 1 such that rotation axis of Joint 2 is perpendicular to center line
        #   D. Calculate JOINT 2 and JOINT 4 rotation such that JOINT 6 lies above desired tool position
        #   E. Calculate JOINT 6 rotation such that tool x axis points downwards

        #   The reason of such complex script is that solving a standard IK for a redundant robot might not give
        #   repeatable results and might return joint positions that do not adhere the definition 4 from above.

        #   ACCESS DH PARAMETERS
        d_0_2 = float(np.copy(self.DH_table[0,0]))
        d_2_4 = float(np.copy(self.DH_table[2,0]))
        d_4_6 = float(np.copy(self.DH_table[4,0]))
        d_6_F = float(np.copy(self.DH_table[6,0]))

        # STEP A. CALCULATE FLANGE POSE

        # basis vectors that define tool orientation (in KUKA base frame)
        if self.center_line[0] >= 0:
            # positive half, z points opposite to center line
            tool_basis_x = - np.copy(self.center_line)
            tool_basis_x_norm = tool_basis_x / np.linalg.norm(tool_basis_x)
        else:
            # negative half, z points along center line
            tool_basis_x = np.copy(self.center_line)
            tool_basis_x_norm = tool_basis_x / np.linalg.norm(tool_basis_x)
        tool_basis_z = np.array([0, 0, -1])
        tool_basis_y = np.cross(tool_basis_z, tool_basis_x_norm)

        # get full home pose of tool (in kuka base frame)
        p_base_tool = np.array([[ws_center[0]], [ws_center[1]], [ws_center[2]]])
        R_base_tool = self.getRotationFromBasis(tool_basis_x_norm, tool_basis_y, tool_basis_z)
        T_base_tool = np.zeros((4,4))
        T_base_tool[0:3, 0:3] = R_base_tool
        T_base_tool[0:3, 3] = p_base_tool[0:,0]
        T_base_tool[3, 3] = 1

        # define flange transform (in tool frame)
        T_tool_flange = np.identity(4)

        T_tool_flange[0, 3] = np.copy(self.tool_thickness)
        T_tool_flange[2, 3] = - np.copy(self.tool_length)

        # calculate desired flange pose (in kuka base frame)
        T_base_flange = np.matmul(T_base_tool, T_tool_flange)
        p_base_flange = T_base_flange[0:3, 3]

        # STEP B. FIXED JOINTS

        joint_3 = 0
        joint_5 = 0
        joint_7 = 0

        # STEP C. CALCULATE JOINT 1 ROTATION

        # check if robot should bend over towards positive or negative half
        if self.center_line[0] >= 0:
            # get rotation axis (always z) and angle
            rot_axis = np.array([0, 0, 1])
            rif_vector = np.array([1, 0, 0])  # Reference vector for the rotation (what I want to reach)
            unit_rif_vector = rif_vector / np.linalg.norm(rif_vector)
            unit_center_line = np.copy(self.center_line) / np.linalg.norm(np.copy(self.center_line))
            dot_product = np.matmul(unit_rif_vector, unit_center_line)
            rot_angle_base = np.arccos(dot_product)

            joint_1 = rot_axis[2] * rot_angle_base

        else:
            # get rotation axis (always z) and angle
            rot_axis = np.array([0, 0, -1])
            rif_vector = np.array([-1, 0, 0])  # Reference vector for the rotation (what I want to reach)
            unit_rif_vector = rif_vector / np.linalg.norm(rif_vector)
            unit_center_line = np.copy(self.center_line) / np.linalg.norm(np.copy(self.center_line))
            dot_product = np.dot(unit_rif_vector, unit_center_line)
            rot_angle_base = np.arccos(dot_product)

            joint_1 = rot_axis[2] * rot_angle_base

        # STEP D. CALCULATE JOINT 2 AND 4 ROTATION

        # joint 6 should lie directly above the flange
        joint_6_pos = p_base_flange - np.array([0, 0, d_6_F])

        # make joint 2 origin, throw error if joint 6 is not reachable
        r = joint_6_pos - np.array([d_0_2, 0, 0])
        dist = np.linalg.norm(r)

        if dist > (d_2_4 + d_4_6):
            np.disp("Tryng to make tool tip and workspace center coincide, but given parameters would lead to non-rachable home pose. Your tool might be too long.")

        # get desired joint 6 position in plane coordinates
        z = r[2]
        x = np.sqrt((np.linalg.norm(r) ** 2) - z ** 2)

        # calculate joint angles (from http://www.cs.columbia.edu/~allen/F15/NOTES/invkin.pdf)
        c = (x ** 2 + z ** 2 - d_2_4 ** 2 - d_4_6 ** 2) / (2 * d_2_4 * d_4_6)
        s = np.sqrt(1 - c ** 2)
        angle_1 = np.arctan2((d_4_6 * s * x + (d_2_4 + d_4_6 * c) * z), ((d_2_4 + d_4_6 * c) * x - d_4_6 * s * z))
        angle_2 = np.arccos(c)

        # adjust solution by Columbia university to KUKA joint direction convention
        joint_2 = -pi/2 + angle_1
        joint_4 = - pi + angle_2

        # STEP E. CALCULATE JOINT 6 ROTATION
        joint_6 = pi - joint_2 + joint_4

        # MIRROR RESULT IF SHOULD BE FACING IN NEGATIVE X DIRECTION
        if self.center_line[0] < 0:
            joint_2 = -joint_2
            joint_4 = -joint_4
            joint_6 = -joint_6

        joints = np.array([joint_1, joint_2, joint_3, joint_4, joint_5, joint_6, joint_7])
        # np.disp("Calculated home position in degree is:")

        for i in range(7):
            string = "Joints:" + str(i) + ": " + str(np.rad2deg(joints[i]))
            # np.disp(string)

        # check if solution exceeds joints limits
        in_bounds, joints = (self.checkJointLimits(joints))
        if in_bounds == 1:  # if "in_bound" is false
            np.disp("Calculated home pose exceeds joint limits. [ERROR: in line 261 of 'GetFunctions.py' file")

        return joints, T_base_tool, T_base_flange

    def getRotationFromBasis(self, basis_x, basis_y, basis_z):
        # COMPUTE ROTATION BETWEEN THE STANDARD BASIS AND THE GIVEN ORTHOGONAL BASIS
        #  STANDARD BASIS = [1,0,0], [0,1,0], [0,0,1]
        #  NEW BASIS : the vectors need to be linearly independent and the basis must be invertible.

        des_size = (np.array([[0], [0], [0]])).shape
        basis_z = basis_z.reshape(-1,1)
        basis_y = basis_y.reshape(-1,1)
        basis_x = basis_x.reshape(-1,1)

        if basis_x.shape != des_size or basis_y.shape != des_size or basis_z.shape != des_size:
            np.disp("Basis Vector need to be column vector. [ERROR]: in line 283 of 'setupROS.py' file.")

        # in case we are given non-orthonormal basis
        basis_x = basis_x / np.linalg.norm(basis_x)
        basis_y = basis_y / np.linalg.norm(basis_y)
        basis_z = basis_z / np.linalg.norm(basis_z)

        # rotation from standard basis to new basis is inverse of new basis as shown in standard basis
        R = np.zeros((3,3))
        R[0:, 0] = basis_x[0:, 0]
        R[0:, 1] = basis_y[0:, 0]
        R[0:, 2] = basis_z[0:, 0]
        R_inv = np.linalg.inv(R)

        return R

    def checkJointLimits(self, joints):

        if len(joints) != 7:
            np.disp("Expecting array of 7 joints values. [ERROR] --> line 39 of 'checkFunctions.py' file")

        clearance = np.deg2rad(5)
        limits = np.array(np.deg2rad([170, 120, 170, 120, 170, 120, 175])) - clearance * np.ones((1, 7))
        limits = limits.reshape(-1)
        in_bounds = 0

        for i in range(7):
            if abs(joints[i]) > limits[i]:
                in_bounds = 1
                print("Joint limit " + str(i + 1) + "exceeded! Given: " + str(joints[i]) + " Allowed: +/- " + str(
                    limits[i]))

        joints = np.clip(joints, -limits, limits)


        return in_bounds, joints

    def checkWSValidity(self):

        ws_verify = "true"
        homePoseDes = np.array(self.home_pose)
        T_BF = self.getPose(homePoseDes)
        T_BT = np.matmul(T_BF,self.T_FT)
        ws_center = T_BT[0:3,3]

        # 1. FEASIBILITY CHECKS
        # point must lie in "positive half" which is defined by center line
        if np.matmul(ws_center,(np.copy(self.center_line))) <= 0:
            np.disp("Error. Desired position not in positive half of robot.")
            ws_verify = "false"

        if ws_center[2] < 0:
            np.disp("Error. Desired position lies below KUKA XY plane.")
            ws_verify = "false"

        # 2. ENFORCE OPENING ANGLE PLANE CONSTRAINTS
        # find direction vector that span workspace planes with KUKA z axis
        OA = np.copy(self.opening_angle)
        R_z_r = np.array([[cos(-OA / 2), - sin(-OA / 2), 0],
                         [sin(-OA / 2), cos(-OA / 2), 0], [0, 0, 1]])
        R_z_l = np.array([[cos(OA / 2), - sin(OA / 2), 0],
                         [sin(OA / 2), cos(OA / 2), 0], [0, 0, 1]])

        plane_r_dir = np.matmul(R_z_r.T, np.copy(self.center_line))
        plane_l_dir = np.matmul(R_z_l.T, np.copy(self.center_line))
        plane_r_dir = plane_r_dir / np.linalg.norm(plane_r_dir)
        plane_l_dir = plane_l_dir / np.linalg.norm(plane_l_dir)

        plane_r_normal = np.cross(plane_r_dir, [0, 0, 1])
        plane_l_normal = -np.cross(plane_l_dir, [0, 0, 1])

        if ws_center.dot(plane_r_normal) > 0:
            np.disp("Enforcing right opening angle constraint,")
            # vector projection
            a1 = ws_center[0:2].dot(plane_r_dir[0: 2]) * plane_r_dir[0: 2]
            a2 = ws_center[0:2] - a1
            ws_center[0:2] = ws_center[0:2] - a2
            ws_verify = "false"

        if ws_center.dot(plane_l_normal) > 0:
            np.disp("Enforcing left opening angle constraint,")
            # vector projection
            a1 = ws_center[0:2].dot(plane_l_dir[0: 2]) * plane_l_dir[0: 2]
            a2 = ws_center[0:2] - a1
            ws_center[0:2] = ws_center[0:2] - a2
            ws_verify = "false"

        # 3. ENFORCE Z PLANE CONSTRAINTS
        if ws_center[2] < self.zLowerLimit:
            ws_center[2] = np.copy(self.zLowerLimit)
            np.disp("Enforcing lower z limit.")
            ws_verify = "false"
        elif ws_center[2] > self.zUpperLimit:
            ws_center[2] = np.copy(self.zUpperLimit)
            np.disp("Enforcing upper z limit.")
            ws_verify = "false"

        # 4. ENFORCE SPHERE CONSTRAINTS
        sphere_origin = np.array([0, 0, np.copy(self.DH_table[0, 0])])
        # translate vector from sphere origin to KUKA origin
        ws_center = ws_center - sphere_origin
        z = ws_center[2]

        # get euclidean distance between origin and desired position
        dist = np.linalg.norm(ws_center)
        arr = np.array([0, 0, z])

        if dist < self.inner_sphere:
            # find desired offset from z axis with pythagoras
            offset = np.sqrt(np.copy(self.inner_sphere)**2 - z**2)
            # find unit vector in x,y plane and translate accordingly
            direction = ws_center - [0, 0, z]
            unit_vector = direction / np.linalg.norm(direction)
            ws_center = (offset * unit_vector) +arr
            np.disp("Enforcing inner sphere constraint.")
            ws_verify = "false"

        elif dist > self.outer_sphere:
            # find desired offset from z axis with pythagoras
            offset = np.sqrt(np.copy(self.outer_sphere) ** 2 - z ** 2)
            # find unit vector in x,y plane and translate accordingly
            direction = ws_center - [0, 0, z]
            unit_vector = direction / np.linalg.norm(direction)
            ws_center = (offset * unit_vector) + arr
            np.disp("Enforcing outer sphere constraint.")
            ws_verify = 'false'

        ws_center = ws_center + sphere_origin

        return ws_center, ws_verify

            # Direct kinematics
    def getPose(self, q):
        Pose = np.copy(np.identity(4))
        DH_table = np.copy(self.DH_table)

        for i in range(7):
            DH_params = np.copy(DH_table[i,:])
            DH_params[1] = DH_params[1] + q[i]
            T_i = self.DH_mat(DH_params)
            Pose = np.matmul(Pose,T_i)

        return Pose

    def DH_mat(self, DH_param):
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




