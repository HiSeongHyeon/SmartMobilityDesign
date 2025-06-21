#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import time, math
import numpy as np
from cv_bridge import CvBridge
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

    # 기본 주행 PID 제어 
    def PID(self, center, kp=0.37, ki=0.001, kd=0.06):
        end_time = time.time()
        dt = end_time - self.start_time
        self.start_time = end_time

        # 도로 중심 320 보다 좌측으로 달릴 수 있도록 error 설정 
        # 340으로 설정 시 정중앙으로 주행 (카메라 편향으로 인하여) 370으로 설정 시 약간 좌측(인코스로 주행)
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

    # 장애물 회피 PID 제어
    def obstacle_PID(self, input, theta, kp=0.41, ki=0.001, kd=0.05):

        end_time = time.time()
        dt = end_time - self.start_time
        self.start_time = end_time

        #차량 중심과 장애물 끝점이 0.3m 거리를 두고 주행할 수 있도록 설정 (300은 스케일 가중치)
        error = (0.3 - input * math.sin(math.radians(theta))) * 300
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

    # 터널 주행 PID 제어
    def tunnel_PID(self, input_left, input_right, kp=0.39, ki=0.005, kd=0.15):
        end_time = time.time()
        dt = end_time - self.start_time
        self.start_time = end_time

        # 좌측과 우측 라이다 값 차이를 error로 사용 (250은 스케일 가중치)
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
