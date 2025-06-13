#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import cv2


from control import XycarControl
from config import mtx, dist, Width, Height
from sensor import Camera, Lidar,lidar_to_mask, undistort_and_birdseye
import numpy as np

# obs or tunnel or crosswalk or stopline


if __name__ == '__main__':
    rospy.init_node('auto_drive')
    camera = Camera()
    lidar = Lidar()
    control = XycarControl()
    control.init_publisher()
    control.init_PID_gain(kp=0.41, ki=0.001, kd=0.05)
    
    # 첫 이미지 수신 대기
    while not rospy.is_shutdown() and (np.sum(camera.raw_image) == 0 or lidar.lidar_points is None):
        rospy.sleep(0.1)

    while not rospy.is_shutdown():
        # obs = is_obstacle_ahead()

        # if is_tunnel():
        #     print("this is tunnel")
        #     in_tunnel()

        # elif obs != 0:
        #     if obs == 1:
        #         print("=== obstacle is right side ===")
        #         right_obstacle_driving()

        #     if obs == 2:
        #         print("=== obstacle is left side ===")
        #         left_obstacle_driving()

        # else:
        lpos, rpos, is_crosswalk = camera.process_calibration_and_birdeye()

        center = (lpos + rpos) / 2
        angle = control.PID(center)
        control.drive(angle, 5)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    print("---------------------------------------------------------")
    print("Program Done")