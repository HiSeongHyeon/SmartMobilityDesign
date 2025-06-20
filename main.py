#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import cv2, time


from control import XycarControl
from config import crosswalk_completed
from sensor import Camera, Lidar
import numpy as np



if __name__ == '__main__':
    rospy.init_node('auto_drive')
    camera = Camera()
    lidar = Lidar()
    control = XycarControl()
    control.init_publisher()


    # 첫 이미지 및 라이다 수신 대기
    while not rospy.is_shutdown() and (np.sum(camera.raw_image) == 0 or lidar.lidar_points is None):
        rospy.sleep(0.1)

    while not rospy.is_shutdown():
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        # 터널 구간 판단
        is_tunnel = lidar.is_tunnel()

        # 장애물 구간 판단
        is_obs = lidar.is_obstacle_ahead()

        # 터널 구간 제어
        if is_tunnel:
            print("this is tunnel")
            left, right = lidar.tunnel_driving()
            angle = control.tunnel_PID(left, right)
            control.drive(angle, 5)
            continue   #continue 문을 통해 일반 직선 및 곡선 구간, 횡단보도 구간, 정지선 구간 판단 로직 스킵

        # 장애물 구간 제어
        elif is_obs:
            if is_obs == 1:
                print("=== obstacle is right side ===")
                distance, theta = lidar.right_obstacle_driving()
                if theta > 90:
                    angle = -control.obstacle_PID(distance, theta)
                else:
                    angle = control.obstacle_PID(distance, theta)
                control.drive(angle, 5)

            if is_obs == 2:
                print("=== obstacle is left side ===")
                distance, theta = lidar.left_obstacle_driving()
                if theta > 90:
                    angle = control.obstacle_PID(distance, theta)
                else:
                    angle = -control.obstacle_PID(distance, theta)
                control.drive(angle, 5)
            continue    #continue 문을 통해 일반 직선 및 곡선 구간, 횡단보도 구간, 정지선 구간 판단 로직 스킵

        # 일반 직선 및 곡선 구간, 횡단보도 구간, 정지선 구간 판단.
        lpos, rpos, is_crosswalk, is_stopline = camera.process_calibration_and_birdeye()

        # 횡단보도 구간 제어
        if(is_crosswalk and not crosswalk_completed):
            print"========== this is crosswalk ========="
            control.drive(0, 0)
            time.sleep(5)
            crosswalk_completed = True
            camera.crosswalk_completed = crosswalk_completed

        # 정지선 구간 제어
        elif(crosswalk_completed and is_stopline):
            print"========= this is stopline ========="
            control.drive(0, 0)
            time.sleep(1)
            break

        # 일반 직선 및 곡선 구간 제어
        else:
            print"========= this is general ========="
            center = (lpos + rpos) / 2
            angle = control.PID(center)
            control.drive(angle, 5)





    rospy.spin()
    print("---------------------------------------------------------")
    print("Program Done")
