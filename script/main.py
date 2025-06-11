#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, LaserScan
from xycar_msgs.msg import xycar_motor
from control import XycarControl
from config import mtx, dist, Width, Height
from sensor import lidar_to_mask, undistort_and_birdseye
import numpy as np

rospy.init_node('auto_drive')
bridge = CvBridge()
control = XycarControl()
control.init_publisher()
cal_mtx, cal_roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (Width, Height), 1, (Width, Height))

def lidar_callback(scan):

    print"B"
    lidar_mask = lidar_to_mask(scan, cal_mtx)
    print"C"
    control.set_lidar_mask(lidar_mask)
    print"D"

def img_callback(data):

    print"a"
    raw_image = bridge.imgmsg_to_cv2(data, "bgr8")
    print"b"
    bird_eye_gray, gray = undistort_and_birdseye(raw_image, cal_mtx, cal_roi)
    if (bird_eye_gray is None or gray is None):
        print("error")
    print"c"
    # if control.lidar_mask is None:
    #     return
    # final_image = cv2.merge([bird_eye_gray, control.lidar_mask])
    # print"d"
    # final_image_16u = cv2.merge([final_image[:,:,0], final_image[:,:,1]]).astype(np.uint16)
    # print"e"
    # ros_image = bridge.cv2_to_imgmsg(final_image_16u, encoding="16UC2")  
    # print"f"
    # image_pub.publish(ros_image)
    bird_eye_gray_msg = bridge.cv2_to_imgmsg(bird_eye_gray, encoding="mono8")
    image_pub.publish(bird_eye_gray_msg)
    gray_msg = bridge.cv2_to_imgmsg(gray, encoding="mono8")
    gray_pub.publish(gray_msg)
    print"g"


if __name__ == '__main__':
    image_pub = rospy.Publisher('/fusion/final_image', Image, queue_size=1)
    gray_pub = rospy.Publisher('/fusion/undistort_gray_image', Image, queue_size=1)
    rospy.Subscriber("/usb_cam/image_raw", Image, img_callback)
    # rospy.Subscriber("/scan", LaserScan, lidar_callback)
    print("image subscriber start")
    rospy.spin()
