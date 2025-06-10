#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, rospkg, time
import numpy as np
import cv2, random, math

from cv_bridge import CvBridge
from sensor_msgs.msg import LaserScan
from xycar_msgs.msg import xycar_motor
from sensor_msgs.msg import Image
import time

import sys
import os
import signal


################################################################################

# Declaration of Global Variables
image = np.empty(shape=[0])
bridge = CvBridge()
pub = None
Width = 640
Height = 480
Offset = 340
Gap = 40
msg = xycar_motor()
lidar_points = None

# system
def signal_handler(sig, frame):
    os.system('killall -9 python rosout')
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

# ROS callback: image
def img_callback(data):
    global image
    image = bridge.imgmsg_to_cv2(data, "bgr8")

# ROS callback: lidar
def callback(data):
    global lidar_points
    lidar_points = data.ranges

# publish xycar_motor msg
def drive(Angle, Speed):
    global pub
    global msg

    msg.angle = Angle
    msg.speed = Speed

    pub.publish(msg)

# CV: draw lines
def draw_lines(img, lines):
    global Offset
    for line in lines:
        x1, y1, x2, y2 = line[0]
        color = (random.randint(0, 255), random.randint(0, 255), random.randint(0, 255))
        img = cv2.line(img, (x1, y1+Offset), (x2, y2+Offset), color, 2)
    return img

# CV: draw rectangle
def draw_rectangle(img, lpos, rpos, offset=0):
    center = (lpos + rpos) / 2

    cv2.rectangle(img, (lpos - 5, 15 + offset),
                       (lpos + 5, 25 + offset),
                       (0, 255, 0), 2)
    cv2.rectangle(img, (rpos - 5, 15 + offset),
                       (rpos + 5, 25 + offset),
                       (0, 255, 0), 2)
    cv2.rectangle(img, (center-5, 15 + offset),
                       (center+5, 25 + offset),
                       (0, 255, 0), 2)
    cv2.rectangle(img, (315, 15 + offset),
                       (325, 25 + offset),
                       (0, 0, 255), 2)
    return img

# CV: left lines, right lines
def divide_left_right(lines):
    global Width

    low_slope_threshold = 0
    high_slope_threshold = 10

    # calculate slope & filtering with threshold
    slopes = []
    new_lines = []

    for line in lines:
        x1, y1, x2, y2 = line[0]
        if x2 - x1 == 0:
            slope = 0
        else:
            slope = float(y2-y1) / float(x2-x1)

        if abs(slope) > low_slope_threshold and abs(slope) < high_slope_threshold:
            slopes.append(slope)
            new_lines.append(line[0])

    # divide lines left to right
    left_lines = []
    right_lines = []

    for j in range(len(slopes)):
        Line = new_lines[j]
        slope = slopes[j]

        x1, y1, x2, y2 = Line

        if (slope < 0) and (x2 < Width/2 - 90):
            left_lines.append([Line.tolist()])
        elif (slope > 0) and (x1 > Width/2 + 90):
            right_lines.append([Line.tolist()])

    #print(left_lines)
    return left_lines, right_lines

# CV: get average m, b of lines
def get_line_params(lines):
    # sum of x, y, m
    x_sum = 0.0
    y_sum = 0.0
    m_sum = 0.0

    size = len(lines)
    if size == 0:
        return 0, 0

    for line in lines:
        x1, y1, x2, y2 = line[0]

        x_sum += x1 + x2
        y_sum += y1 + y2
        m_sum += float(y2 - y1) / float(x2 - x1)

    x_avg = x_sum / (size * 2)
    y_avg = y_sum / (size * 2)
    m = m_sum / size
    b = y_avg - m * x_avg

    return m, b

# CV: get lpos, rpos
def get_line_pos(img, lines, left=False, right=False):
    global Width, Height
    global Offset, Gap

    m, b = get_line_params(lines)

    if m == 0 and b == 0:
        if left:
            pos = 0
        if right:
            pos = Width
    else:
        y = Gap / 2
        pos = (y - b) / m

        b += Offset
        x1 = (Height - b) / float(m)
        x2 = ((Height/2) - b) / float(m)

        cv2.line(img, (int(x1), Height), (int(x2), (Height/2)), (255, 0,0), 3)

    return img, int(pos)

# CV: show image and return lpos, rpos
def process_image(frame):
    global Width
    global Offset, Gap

    # gray
    gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)

    # blur
    kernel_size = 5
    blur_gray = cv2.GaussianBlur(gray,(kernel_size, kernel_size), 0)

    # canny edge
    low_threshold = 60
    high_threshold = 70
    edge_img = cv2.Canny(np.uint8(blur_gray), low_threshold, high_threshold)

    # HoughLinesP
    roi = edge_img[Offset : Offset+Gap, 0 : Width]
    all_lines = cv2.HoughLinesP(roi,1,math.pi/180,30,30,10)

    # divide left, right lines
    if all_lines is None:
        return 0, 640
    left_lines, right_lines = divide_left_right(all_lines)

    # get center of lines
    frame, lpos = get_line_pos(frame, left_lines, left=True)
    frame, rpos = get_line_pos(frame, right_lines, right=True)

    # draw lines
    frame = draw_lines(frame, left_lines)
    frame = draw_lines(frame, right_lines)
    frame = cv2.line(frame, (230, 235), (410, 235), (255,255,255), 2)

    # draw rectangle
    frame = draw_rectangle(frame, lpos, rpos, offset=Offset)
    #roi2 = cv2.cvtColor(roi, cv2.COLOR_GRAY2BGR)
    #roi2 = draw_rectangle(roi2, lpos, rpos)

    # show image
    cv2.imshow('calibration', frame)

    return lpos, rpos

################################################################################

# PID Control: basic driving
def PID(input_data, kp=0.41, ki=0.001, kd=0.05):
    global start_time, end_time, prev_error, i_error
    end_time = time.time()
    dt = end_time - start_time
    start_time = end_time

    error = 320 - input_data
    derror = error - prev_error
    p_error = kp * error
    i_error = i_error + ki * error * dt
    d_error = kd * derror / dt

    output = p_error + i_error + d_error
    prev_error = error

    if output > 50:
        output = 50
    elif output < -50:
        output = -50

    return -output

# PID Control: avoiding obstacle
def obstacle_PID(input, theta, kp=0.41, ki=0.001, kd=0.05):
    global obstacle_start_time, end_time, obstacle_prev_error, obstacle_i_error
    end_time = time.time()
    dt = end_time - obstacle_start_time
    obstacle_start_time = end_time

    error = (0.5 - input * math.sin(math.radians(theta))) * 150
    derror = error - obstacle_prev_error
    p_error = kp * error
    obstacle_i_error = obstacle_i_error + ki * error * dt
    d_error = kd * derror / dt
    output = p_error + obstacle_i_error + d_error
    obstacle_prev_error = error

    if output > 50:
        output = 50
    elif output < -50:
        output = -50

    return -output

# PID Control: driving tunnel
def tunnel_PID(input_left, input_right, kp=0.4, ki=0.0005, kd=0.15):
    global tunnel_start_time, end_time, tunnel_prev_error, tunnel_i_error
    end_time = time.time()
    dt = end_time - tunnel_start_time
    tunnel_start_time = end_time

    error = (input_right - input_left) * 300
    derror = error - tunnel_prev_error
    p_error = kp * error
    tunnel_i_error = tunnel_i_error + ki * error * dt
    d_error = kd * derror / dt
    output = p_error + tunnel_i_error + d_error
    tunnel_prev_error = error

    if output > 50:
        output = 50
    elif output < -50:
        output = -50

    return output

################################################################################

# Recognition: obstacle
def is_obstacle_ahead(threshold=0.5, check_range=60, count_limit=6):
    global lidar_points
    countright = 0
    countleft = 0

    for deg in range(check_range+1):
        if 0.1 < lidar_points[deg+180] <= threshold:
            countright += 1
        if 0.1 < lidar_points[180-deg] <= threshold:
            countleft += 1

    if countright > count_limit and countleft < count_limit:
        return 1
    elif countright < count_limit and countleft > count_limit:
        return 2
    else:
        return 0

# Recognition: tunnel
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

################################################################################

# Performing: when obastacle is right side
def right_obstacle_driving():
    global lidar_points
    rtn = list()
    distance = 0
    count = 0
    for i in range(300):
        if not np.isinf(lidar_points[i+180]):
            if 0.1 < lidar_points[i+180] < 0.4:
                rtn.append(lidar_points[i+180])
            else:
                rtn.append(0)
        else:
            rtn.append(0)
    for i in range(len(rtn)):
	    if rtn[i] == 0:
	        continue
	    else:
	        distance += rtn[i]
            count += 1
	        if count == 5:
		        break
    angle = obstacle_PID(distance / 5, i/2)
    print("avg distance and theta = ", distance / 5, i/2)
    print("left angle = ", angle)
    drive(angle, 5)

# Performing: when obastacle is left side
def left_obstacle_driving():
    global lidar_points
    rtn = list()
    distance = 0
    count = 0
    for i in range(300):
        if not np.isinf(lidar_points[180-i]):
            if 0.1 < lidar_points[180-i] <0.4:
                rtn.append(lidar_points[180-i])
            else:
                rtn.append(0)
        else:
            rtn.append(0)
    for i in range(len(rtn)):
	    if rtn[i] == 0:
	        continue
	    else:
	        distance += rtn[i]
            count += 1
	        if count == 5:
		        break
    angle = -obstacle_PID(distance / 5, i/2)
    print("avg distance and theta = ", distance / 5, i/2)
    print("right angle = ", angle)
    drive(angle, 5)

# Performing: tunnel
def in_tunnel():
    global lidar_points
    left = lidar_points[0]
    right = lidar_points[360]
    angle = tunnel_PID(left, right)
    speed = 5
    drive(angle, speed)

################################################################################

# main function
def start():
    global pub
    global image
    global cap
    global Width, Height

    rospy.init_node('auto_drive')
    pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
    rospy.Subscriber('/scan', LaserScan, callback, queue_size=1)

    image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, img_callback)
    print "---------- Xycar A2 v1.0 ----------"
    rospy.sleep(2)

    while lidar_points is None:
        continue

    while True:

        obs = is_obstacle_ahead()

	    if is_tunnel():
            print("this is tunnel")
            in_tunnel()

        elif obs != 0:
            if obs == 1:
                print("=== obstacle is right side ===")
                right_obstacle_driving()

            if obs == 2:
                print("=== obstacle is left side ===")
                left_obstacle_driving()

        else:
            print("general general general")
            while not image.size == (640*480*3):
                continue

            lpos, rpos = process_image(image)

            center = (lpos + rpos) / 2
            angle = PID(center)
            drive(angle, 5)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    rospy.spin()

if __name__ == '__main__':
    # Declaring PID init value
    i_error = 0.0
    tunnel_i_error = 0.0
    obstacle_i_error = 0.0
    prev_error = 0.0
    tunnel_prev_error = 0.0
    obstacle_prev_error = 0.0
    start_time = time.time()
    tunnel_start_time = time.time()
    obstacle_start_time = time.time()

    start()

