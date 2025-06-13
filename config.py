#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np

Width, Height = 640, 480

mtx = np.array([[340.876013, 0.000000, 333.212353],
                [0.000000, 341.790625, 241.433953],
                [0.000000, 0.000000, 1.000000]])
dist = np.array([-0.302220, 0.069858, 0.000204, -0.002379, 0.000000])
src_pts = np.float32([[200, 300], [440, 300], [100, 480], [540, 480]])
dst_pts = np.float32([[200, 0], [440, 0], [200, 480], [440, 480]])
Offset = 100
Gap = 40
Width_Offset = 40


# Bird eye ROI 설정
bird_eye_roi_x_start = 200
bird_eye_roi_x_end = 440
bird_eye_roi_y_start = 20
bird_eye_roi_y_end = 180


R_lidar2cam = np.eye(3)
T_lidar2cam = np.array([[0], [-5], [10]])


Debug = True
stop_completed = False