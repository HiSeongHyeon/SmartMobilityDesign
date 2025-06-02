#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, rospkg
import numpy as np
import cv2, random, math
from cv_bridge import CvBridge
from xycar_msgs.msg import xycar_motor
from sensor_msgs.msg import Image
import time
import sys
import os
import signal
import csv

image = np.empty(shape=[0])
bridge = CvBridge()
pub = None
Width = 640
Height = 480
Offset = 340
Gap = 40

i_error = 0.0
prev_error = 0.0
start_time = time.time()
init_time = start_time

# Python 2용 파일 저장: newline 인자 제거, 'wb' 모드 사용
script_dir = os.path.dirname(os.path.abspath(__file__))
csv_path = os.path.join(script_dir, "tracking_error_log.csv")
log_file = open(csv_path, 'wb')
log_writer = csv.writer(log_file)
log_writer.writerow(["Time(s)", "Target", "Center", "Error", "PID_Output"])

def signal_handler(sig, frame):
    global log_file
    print("Shutting down...")
    log_file.close()
    os.system('killall -9 python rosout')
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

def PID(input_data, kp, ki, kd):
    global start_time, prev_error, i_error, log_writer, init_time
    end_time = time.time()
    dt = end_time - start_time
    start_time = end_time

    error = 320 - input_data
    derror = error - prev_error
    p_error = kp * error
    i_error += ki * error * dt
    d_error = kd * derror / dt

    output = p_error + i_error + d_error
    prev_error = error

    if output > 50:
        output = 50
    elif output < -50:
        output = -50

    elapsed = round(end_time - init_time, 4)
    print("[%0.4fs] Target=320 | Center=%d | Error=%.2f | PID Output=%.2f" % (elapsed, input_data, error, output))
    log_writer.writerow([elapsed, 320, input_data, round(error, 4), round(output, 4)])

    return -output

def img_callback(data):
    global image
    image = bridge.imgmsg_to_cv2(data, "bgr8")

def drive(Angle, Speed): 
    global pub
    msg = xycar_motor()
    msg.angle = Angle
    msg.speed = Speed
    pub.publish(msg)

def draw_lines(img, lines):
    global Offset
    for line in lines:
        x1, y1, x2, y2 = line[0]
        color = (random.randint(0, 255), random.randint(0, 255), random.randint(0, 255))
        img = cv2.line(img, (x1, y1+Offset), (x2, y2+Offset), color, 2)
    return img

def draw_rectangle(img, lpos, rpos, offset=0):
    center = (lpos + rpos) // 2
    cv2.rectangle(img, (lpos - 5, 15 + offset), (lpos + 5, 25 + offset), (0, 255, 0), 2)
    cv2.rectangle(img, (rpos - 5, 15 + offset), (rpos + 5, 25 + offset), (0, 255, 0), 2)
    cv2.rectangle(img, (center-5, 15 + offset), (center+5, 25 + offset), (0, 255, 0), 2)
    cv2.rectangle(img, (315, 15 + offset), (325, 25 + offset), (0, 0, 255), 2)
    return img

def divide_left_right(lines):
    global Width
    low_slope_threshold = 0
    high_slope_threshold = 10
    slopes = []
    new_lines = []

    for line in lines:
        x1, y1, x2, y2 = line[0]
        if x2 - x1 == 0:
            slope = 0
        else:
            slope = float(y2 - y1) / float(x2 - x1)
        if abs(slope) > low_slope_threshold and abs(slope) < high_slope_threshold:
            slopes.append(slope)
            new_lines.append(line[0])

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

    return left_lines, right_lines

def get_line_params(lines):
    x_sum, y_sum, m_sum = 0.0, 0.0, 0.0
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

def get_line_pos(img, lines, left=False, right=False):
    global Width, Height, Offset, Gap
    m, b = get_line_params(lines)
    if m == 0 and b == 0:
        pos = 0 if left else Width
    else:
        y = Gap / 2
        pos = (y - b) / m
        b += Offset
        x1 = (Height - b) / float(m)
        x2 = ((Height/2) - b) / float(m)
        cv2.line(img, (int(x1), Height), (int(x2), int(Height/2)), (255, 0, 0), 3)
    return img, int(pos)

def process_image(frame):
    global Width, Offset, Gap
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blur_gray = cv2.GaussianBlur(gray, (5, 5), 0)
    edge_img = cv2.Canny(np.uint8(blur_gray), 60, 70)
    roi = edge_img[Offset : Offset+Gap, 0 : Width]
    all_lines = cv2.HoughLinesP(roi, 1, math.pi/180, 30, 30, 10)

    if all_lines is None:
        return 0, 640
    left_lines, right_lines = divide_left_right(all_lines)
    frame, lpos = get_line_pos(frame, left_lines, left=True)
    frame, rpos = get_line_pos(frame, right_lines, right=True)
    frame = draw_lines(frame, left_lines)
    frame = draw_lines(frame, right_lines)
    frame = cv2.line(frame, (230, 235), (410, 235), (255, 255, 255), 2)
    frame = draw_rectangle(frame, lpos, rpos, offset=Offset)
    cv2.imshow('calibration', frame)
    return lpos, rpos

def start():
    global pub, image
    rospy.init_node('auto_drive')
    pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
    image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, img_callback)
    print("---------- Xycar A2 v1.0 with Logging (Python 2) ----------")
    rospy.sleep(2)

    while True:
        while not image.size == (640*480*3):
            continue
        lpos, rpos = process_image(image)
        center = (lpos + rpos) // 2
        angle = PID(center, 0.35, 0.002, 0.15)
        drive(angle, 5)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    rospy.spin()

if __name__ == '__main__':
    start()
