#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import time, math
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from xycar_msgs.msg import xycar_motor

class XycarControl:
    def __init__(self):
        self.bridge = CvBridge()
        self.i_error = 0.0
        self.prev_error = 0.0
        self.start_time = time.time()
        self.i_error_tunnel = 0.0
        self.prev_error_tunnel = 0.0
        self.start_time_tunnel = time.time()
        self.stop_completed = False
        self.crosswalk_detected = False
        self.lidar_mask = None
        self.pub = None  
        self.kp =None
        self.ki = None
        self.kd = None

    def init_publisher(self, pub_topic='xycar_motor'):
        self.pub = rospy.Publisher(pub_topic, xycar_motor, queue_size=1)

    def init_PID_gain(self, kp, ki, kd):
        self.kp, self.ki, self.kd = kp, ki, kd

    def PID(self, center, kp=0.37, ki=0.001, kd=0.06):
        end_time = time.time()
        dt = end_time - self.start_time
        self.start_time = end_time

        error = 340 + 30 - center
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

    # PID Control: avoiding obstacle
    def obstacle_PID(self, input, theta, kp=0.41, ki=0.001, kd=0.05):
        # self.i_error = 0.0 두 전역 변수 대신에 지역변수 그냥쓰면 안되나
        # self.prev_error = 0.0
        # obstacle start time
        end_time = time.time()
        dt = end_time - self.start_time
        self.start_time = end_time

        error = (0.5 - input * math.sin(math.radians(theta))) * 150
        derror = error - self.prev_error
        p_error = kp * error
        self.i_error = self.i_error + ki * error * dt
        d_error = kd * derror / dt
        output = p_error + self.i_error + d_error
        self.prev_error = error

        if output > 50:
            output = 50
        elif output < -50:
            output = -50

        return -output

    # PID Control: driving tunnel
    def tunnel_PID(self, input_left, input_right, kp=0.4, ki=0.0005, kd=0.15):
        end_time = time.time()
        dt = end_time - self.start_time
        self.start_time = end_time

        error = (input_right - input_left) * 250
        derror = error - self.prev_error
        p_error = kp * error
        self.i_error_tunnel = self.i_error_tunnel + ki * error * dt
        d_error = kd * derror / dt
        output = p_error + self.i_error_tunnel + d_error
        self.prev_error = error

        if output > 50:
            output = 50
        elif output < -50:
            output = -50

        return output
    
    def drive(self, angle, speed):
        if self.pub is None:
            rospy.logerr("Publisher is not initialized")
            return
        msg = xycar_motor()
        msg.angle = angle
        msg.speed = speed
        self.pub.publish(msg)
