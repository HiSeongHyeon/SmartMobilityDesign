#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import cv2


from control import XycarControl
from config import mtx, dist, Width, Height, stop_completed
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
        obs = lidar.is_obstacle_ahead()

        if is_tunnel():
            print("this is tunnel")
            left = lidar.lidar_points[0]
            right = lidar.lidar_points[360]
            angle = tunnel_PID(left, right)
            drive(angle, 5)

        elif obs != 0:
            if obs == 1:
                print("=== obstacle is right side ===")
                distance, theta = lidar.right_obstacle_driving()
                if theta > 90:
                    angle = -obstacle_PID(distance, theta)
                else:
                    angle = obstacle_PID(distance, theta)
                print("avg distance and theta = ", distance, theta)
                print("left angle = ", angle)
                drive(angle, 5)

            if obs == 2:
                print("=== obstacle is left side ===")
                distance, theta = lidar.left_obstacle_driving()
                if theta > 90:
                    angle = obstacle_PID(distance, theta)
                else:
                    angle = -obstacle_PID(distance, theta)
                print("avg distance and theta = ", distance, theta)
                print("right angle = ", angle)
                drive(angle, 5)

        # else:
        lpos, rpos, is_crosswalk = camera.process_calibration_and_birdeye()

        center = (lpos + rpos) / 2
        angle = control.PID(center)
        control.drive(angle, 5)

        if(is_crosswalk and not stop_completed):
            control.drive(0, 0)
            time.sleep(5)
            stop_completed = True

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    print("---------------------------------------------------------")
    print("Program Done")
