#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, LaserScan
from xycar_msgs.msg import xycar_motor
import cv2
import time
from config import mtx, dist, Width, Height
from sensor import lidar_to_mask, undistort_and_birdseye
from control import PID, drive, process_and_control

bridge = CvBridge()
lidar_mask = None
final_image = None

def lidar_callback(scan):
    global lidar_mask
    cal_mtx, cal_roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (Width, Height), 1, (Width, Height))
    lidar_mask = lidar_to_mask(scan, cal_mtx)

def img_callback(data):
    global final_image, lidar_mask
    cal_mtx, cal_roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (Width, Height), 1, (Width, Height))
    raw_image = bridge.imgmsg_to_cv2(data, "bgr8")
    bird_eye_gray = undistort_and_birdseye(raw_image, cal_mtx, cal_roi)
    if lidar_mask is None:
        return
    final_image = cv2.merge([bird_eye_gray, lidar_mask])


if __name__ == '__main__':
    rospy.init_node('fusion_birdeye_lane_pid')

    pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
    rospy.Subscriber("/usb_cam/image_raw", Image, img_callback)
    rospy.Subscriber("/scan", LaserScan, lidar_callback)
    process_and_control(pub)
