
# Python 2/3 compatibility
from __future__ import print_function

import numpy as np
import cv2
import json

# built-in modules
import os
import sys
from glob import glob

if __name__ == '__main__':

    files = './ChessBoard0.png'
    # files = './calibration/realsense/*.jpg'
    name = './ChessBoard0'

    # pattern property
    square_size = 0.024
    pattern_width = 9
    pattern_height = 6

    pattern_size = (pattern_width, pattern_height)
    pattern_points = np.zeros((np.prod(pattern_size), 3), np.float32)
    pattern_points[:, :2] = np.indices(pattern_size).T.reshape(-1, 2)
    pattern_points *= square_size

    obj_points = []
    img_points = []
    img_names = glob(files)
    img_names_undistort = []
    h, w = 0, 0
    for fn in img_names:
        print('processing %s... ' % fn, end='')
        img = cv2.imread(fn, 0)
        img = cv2.flip(img, 0)
        if img is None:
            print("Failed to load", fn)
            continue

        h, w = img.shape[:2]
        found, corners = cv2.findChessboardCorners(img, pattern_size)
        if found:
            term = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_COUNT, 30, 0.1)
            cv2.cornerSubPix(img, corners, (5, 5), (-1, -1), term)

            cv2.drawChessboardCorners(img, pattern_size, corners, found)
            cv2.imshow('detected', img)
            cv2.waitKey(1)
        else:
            print('chessboard not found')
            continue
        img_points.append(corners.reshape(-1, 2))
        obj_points.append(pattern_points)
        print('ok')

    # calculate camera distortion
    rms, camera_matrix, dist_coefs, rvecs, tvecs = cv2.calibrateCamera(obj_points, img_points, (w, h), None, None, flags=cv2.CALIB_FIX_K3)

    print("\nRMS:", rms)
    print("camera matrix:\n", camera_matrix)
    print("distortion coefficients: ", dist_coefs.ravel())

    data = {"camera_matrix": camera_matrix.tolist(), "dist_coeff": dist_coefs.tolist(), "height": h, "width": w}
    jname = name + ".json"
    with open(jname, "w") as f:
        json.dump(data, f)

    cv2.destroyAllWindows()



