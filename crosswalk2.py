#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import cv2, random, math, time
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String
from xycar_msgs.msg import xycar_motor

image = np.empty(shape=[0])
bridge = CvBridge()
pub = None
info_pub = None
Width = 640
Height = 480
Offset = 240
Gap = 40

# PID control variables
i_error = 0.0
prev_error = 0.0
start_time = time.time()

# Detection flags
crosswalk_detected = False
stop_completed = False
horizontal_line_detected = False

def draw_lines(img, lines):
    global Offset
    for line in lines:
        x1, y1, x2, y2 = line[0]
        color = (random.randint(0, 255), random.randint(0, 255), random.randint(0, 255))
        cv2.line(img, (x1, y1+Offset), (x2, y2+Offset), color, 2)
    return img

def draw_rectangle(img, lpos, rpos, offset=0):
    center = (lpos + rpos) // 2
    cv2.rectangle(img, (lpos - 5, 15 + offset), (lpos + 5, 25 + offset), (0, 255, 0), 2)
    cv2.rectangle(img, (rpos - 5, 15 + offset), (rpos + 5, 25 + offset), (0, 255, 0), 2)
    cv2.rectangle(img, (center - 5, 15 + offset), (center + 5, 25 + offset), (0, 255, 0), 2)
    cv2.rectangle(img, (315, 15 + offset), (325, 25 + offset), (0, 0, 255), 2)
    return img

def PID(input_data, kp, ki, kd):
    global start_time, prev_error, i_error
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

    return max(min(-output, 50), -50)

def drive(angle, speed):
    msg = xycar_motor()
    msg.angle = angle
    msg.speed = speed
    pub.publish(msg)

def img_callback(data):
    global image
    image = bridge.imgmsg_to_cv2(data, "bgr8")

def count_lines_by_slope(lines, low, high):
    count = 0
    for x1, y1, x2, y2 in lines:
        if x2 - x1 == 0:
            slope = float('inf')
        else:
            slope = float(y2 - y1) / float(x2 - x1)
        if low <= abs(slope) <= high:
            count += 1
    return count

def classify_line_region(frame):
    global crosswalk_detected, stop_completed, horizontal_line_detected

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    edge = cv2.Canny(blur, 60, 70)
    roi = edge[Offset:Offset+Gap, :Width]
    lines = cv2.HoughLinesP(roi, 1, math.pi/180, 30, minLineLength=30, maxLineGap=10)

    debug_img = frame.copy()

    if lines is not None:
        lines = [line[0] for line in lines]
        lines_reshaped = [[x1, y1, x2, y2] for x1, y1, x2, y2 in lines]
        draw_lines(debug_img, [[line] for line in lines_reshaped])

        horizontal_count = count_lines_by_slope(lines_reshaped, 0.0, 0.1)
        diagonal_count = count_lines_by_slope(lines_reshaped, 0.3, 1.0)
        vertical_count = count_lines_by_slope(lines_reshaped, 2.0, float('inf'))

        horizontal_line_detected = horizontal_count >= 1

        result = "none"
        if horizontal_line_detected:
            if vertical_count >= 6:
                crosswalk_detected = True
                result = "crosswalk"
            elif diagonal_count >= 3:
                result = "stop_line"
            else:
                result = "start_line"

        cv2.putText(debug_img, "Detected: {}".format(result), (30, 40), cv2.FONT_HERSHEY_SIMPLEX,
                    1, (0, 255, 255), 2)

        debug_img = draw_rectangle(debug_img, 230, 410, offset=Offset)

    else:
        result = "none"
        cv2.putText(debug_img, "No lines detected", (30, 40), cv2.FONT_HERSHEY_SIMPLEX,
                    1, (0, 0, 255), 2)

    cv2.imshow("Road Line Detection", debug_img)
    return result

def start():
    global pub, info_pub, image, crosswalk_detected, stop_completed
    rospy.init_node('auto_drive')
    pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
    info_pub = rospy.Publisher('xycar_info', String, queue_size=1)
    rospy.Subscriber("/usb_cam/image_raw", Image, img_callback)
    print("---------- Xycar A2 start ----------")
    rospy.sleep(2)

    while not rospy.is_shutdown():
        if image.size != (640 * 480 * 3):
            continue

        result = classify_line_region(image)

        if result == "crosswalk" and not stop_completed:
            print("crosswalk")
            drive(0, 0)
            time.sleep(5)
            stop_completed = True
            crosswalk_detected = False
        elif result == "stop_line":
            print("stop line is detected")
        elif result == "start_line":
            print("start line is detected")

        center = Width // 2
        angle = PID(center, 0.45, 0.0007, 0.25)
        drive(angle, 5)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

if __name__ == '__main__':
    start()
