#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, time
import numpy as np
from sensor_msgs.msg import LaserScan
from xycar_msgs.msg import xycar_motor

motor_msg = xycar_motor()
lidar_points = None

def callback(data):
    global lidar_points
    lidar_points = data.ranges

def drive(angle, speed):
    global motor_msg
    motor_msg.angle = angle
    motor_msg.speed = speed
    pub.publish(motor_msg)


def PID(input_left, input_right, kp, ki, kd):
    global start_time, end_time, prev_error, i_error
    end_time = time.time()
    dt = end_time - start_time
    start_time = end_time

    error = input_right*1000 - input_left*1000
    derror = error - prev_error
    p_error = kp * error
    i_error = i_error + ki * error * dt
    d_error = kd * derror / dt
    print(error)
    output = p_error + i_error + d_error
    print("output=", output)
    prev_error = error

    if output > 50:
        output = 50
    elif output < -50:
        output = -50

    return output


def is_tunnel(threshold=0.5, check_range=10, count_limit=5):
    global lidar_points
    count1 = 0
    count2 = 0
    for deg in range(check_range + 1):
        if np.isinf(lidar_points[0]) or np.isinf(lidar_points[360]):
            continue
        else:
            if 0.01 < lidar_points[0] <= threshold:
                count1 += 1
            if 0.01 < lidar_points[360] <= threshold:
                count2 += 1
    return (count1 > count_limit) and (count2 > count_limit)

def in_tunnel():
    global lidar_points
    left = lidar_points[0]
    right = lidar_points[360]
    angle = PID(left, right, 0.35, 0.0005, 0.15)
    speed = 5
    drive(angle, speed)

def init():
    global pub

    rospy.init_node('tunnel')
    rospy.Subscriber('/scan', LaserScan, callback, queue_size=1)
    pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)

    rospy.sleep(2)

    while lidar_points is None:
        continue


    while True:
        if is_tunnel():
            print("is tunnel")
            in_tunnel()
        else:
            print("is not tunnel")
            drive(0, 5)
    rospy.spin()


if __name__ == "__main__":

    i_error = 0.0
    prev_error = 0.0
    start_time = time.time()

    init()



