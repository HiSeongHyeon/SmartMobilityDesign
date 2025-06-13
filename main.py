#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import cv2, time


from control import XycarControl
from config import mtx, dist, Width, Height, crosswalk_completed
from sensor import Camera, Lidar
import numpy as np

# obs or tunnel or crosswalk or stopline


if __name__ == '__main__':
    rospy.init_node('auto_drive')
    camera = Camera()
    lidar = Lidar()
    control = XycarControl()
    control.init_publisher()

    # 첫 이미지 수신 대기
    while not rospy.is_shutdown() and (np.sum(camera.raw_image) == 0 or lidar.lidar_points is None):
        rospy.sleep(0.1)

    while not rospy.is_shutdown():
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break


        # 장애물 구간 판단
        is_obs = lidar.is_obstacle_ahead()
        # 터널 구간 판단
        is_tunnel = lidar.is_tunnel()



        # 터널 구간 제어
        if is_tunnel:
            print("this is tunnel")
            left, right = lidar.tunnel_driving()
            angle = control.tunnel_PID(left, right)
            control.drive(angle, 5)
            continue

        # 장애물 구간 제어
        elif is_obs:
            if is_obs == 1:
                print("=== obstacle is right side ===")
                distance, theta = lidar.right_obstacle_driving()
                if theta > 90:
                    angle = -control.obstacle_PID(distance, theta)
                else:
                    angle = control.obstacle_PID(distance, theta)
                print("avg distance and theta = ", distance, theta)
                print("left angle = ", angle)
                control.drive(angle, 5)

            if is_obs == 2:
                print("=== obstacle is left side ===")
                distance, theta = lidar.left_obstacle_driving()
                if theta > 90:
                    angle = control.obstacle_PID(distance, theta)
                else:
                    angle = -control.obstacle_PID(distance, theta)
                print("avg distance and theta = ", distance, theta)
                print("right angle = ", angle)
                control.drive(angle, 5)
            continue
            
        # 일반 직선 곡선 구간, 횡단보도 구간, 정지선 구간 판단.
        lpos, rpos, is_crosswalk, is_stopline = camera.process_calibration_and_birdeye()

        
        if(is_crosswalk and not crosswalk_completed):
            print"========== this is crosswalk ========="
            control.drive(0, 0)
            time.sleep(5)
            crosswalk_completed = True
        
        elif(crosswalk_completed and is_stopline):
            print"========= this is stopline ========="
            control.drive(0, 0)
            time.sleep(1)

            break

        else:
            center = (lpos + rpos) / 2
            print"center",center
            angle = control.PID(center)
            control.drive(angle, 5)





    rospy.spin()
    print("---------------------------------------------------------")
    print("Program Done")
