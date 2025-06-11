#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import time
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from xycar_msgs.msg import xycar_motor
from line import classify_line_region_and_lane, plot_lane_detection

class XycarControl:
    def __init__(self):
        self.bridge = CvBridge()
        self.i_error = 0.0
        self.prev_error = 0.0
        self.start_time = time.time()
        self.stop_completed = False
        self.crosswalk_detected = False
        self.lidar_mask = None
        self.pub = None
        rospy.Subscriber('/fusion/final_image', Image, self.image_callback)
        rospy.Subscriber('/fusion/undistort_gray_image', Image, self.gray_image_callback)
    def init_publisher(self, pub_topic='xycar_motor'):
        self.pub = rospy.Publisher(pub_topic, xycar_motor, queue_size=1)

    def PID(self, center, kp, ki, kd):
        end_time = time.time()
        dt = end_time - self.start_time
        self.start_time = end_time

        error = 320 - center
        derror = error - self.prev_error
        p_error = kp * error
        self.i_error += ki * error * dt
        d_error = kd * derror / dt if dt > 0 else 0

        output = p_error + self.i_error + d_error
        self.prev_error = error

        if output > 50:
            output = 50
        elif output < -50:
            output = -50

        return -output

    def drive(self, angle, speed):
        if self.pub is None:
            rospy.logerr("Publisher is not initialized")
            return
        msg = xycar_motor()
        msg.angle = angle
        msg.speed = speed
        self.pub.publish(msg)

    def image_callback(self, msg):
        try:
            print("image subsub")
            # final_image = self.bridge.imgmsg_to_cv2(msg, "16UC2")
            # gray_channel = final_image[:,:,0].astype(np.uint8)
            # lidar_channel = final_image[:,:,1].astype(np.uint8)
            # gray_channel = self.bridge.imgmsg_to_cv2(msg,'mono8')
            # result, lpos, rpos, left_lines, right_lines = classify_line_region_and_lane(gray_channel)

            # plot_lane_detection(gray_channel, result, lpos, rpos, left_lines, right_lines, lidar_channel,False, window_name="Fusion Visualization")
            # center = int((lpos + rpos) / 2.0)

            # if result == "crosswalk" and not self.stop_completed:
            #     print("crosswalk")
            #     self.drive(0, 0)
            #     time.sleep(5)
            #     self.stop_completed = True
            #     self.crosswalk_detected = False
            # elif result == "stop_line":
            #     print("stop line is detected")
            # elif result == "start_line":
            #     print("start line is detected")

            # angle = self.PID(center, 0.45, 0.0007, 0.25)
            # self.drive(angle, 5)
        except Exception as e:
            rospy.logerr("Image processing error: %s", str(e))

    def gray_image_callback(self, msg):
        try:
            print("gray callback")
            # cv_bridge로 ROS Image 메시지를 OpenCV 이미지로 변환
            gray_image = self.bridge.imgmsg_to_cv2(msg,'mono8')
            print"k"
            result, lpos, rpos, left_lines, right_lines = classify_line_region_and_lane(gray_image)
            print
            plot_lane_detection(gray_image, result, lpos, rpos, left_lines, right_lines, None, window_name="Gray Visualization")
            center = int((lpos + rpos) / 2.0)

            if result == "crosswalk" and not self.stop_completed:
                print("crosswalk")
                self.drive(0, 0)
                time.sleep(5)
                self.stop_completed = True
                self.crosswalk_detected = False
            elif result == "stop_line":
                print("stop line is detected")
            elif result == "start_line":
                print("start line is detected")

            angle = self.PID(center,kp=0.41, ki=0.001, kd=0.05)
            self.drive(angle, 5)
            print"end"
        except Exception as e:
            rospy.logerr("Image processing error: %s", str(e))

    def set_lidar_mask(self, lidar_mask):
        self.lidar_mask = lidar_mask
