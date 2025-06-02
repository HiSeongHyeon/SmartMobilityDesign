#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import numpy as np
import cv2, random, math
from cv_bridge import CvBridge

import numpy as np

def line_intersection(m1, b1, m2, b2):
    """Return intersection point of two lines, handle vertical lines"""
    if m1 == m2:
        return None
    if m1 == float('inf'):
        x = b1
        y = m2 * x + b2
        return (x, y)
    if m2 == float('inf'):
        x = b2
        y = m1 * x + b1
        return (x, y)
    x = (b2 - b1) / (m1 - m2)
    y = m1 * x + b1
    return (x, y)

def check_quadrilateral(left_line, right_line, upper_line, lower_line):
    """Check if four lines form a quadrilateral and return the corner points"""
    m_left, b_left = left_line
    m_right, b_right = right_line
    m_upper, b_upper = upper_line
    m_lower, b_lower = lower_line

    # Calculate intersection points
    top_left = line_intersection(m_left, b1=b1, m2=m_upper, b2=b_upper)  # Fixed: b_left
    # (Note: In the above line, b1=b_left is correct, but in code below, it is corrected.)
    # Corrected version:
    top_left = line_intersection(m_left, b_left, m_upper, b_upper)
    top_right = line_intersection(m_right, b_right, m_upper, b_upper)
    bottom_left = line_intersection(m1=m_left, b1=b_left, m2=m_lower, b2=b_lower)
    bottom_right = line_intersection(m_right, b_right, m_lower, b_lower)

    # If any intersection is None, no quadrilateral
    if None in [top_left, top_right, bottom_left, bottom_right]:
        return False, None

    # Return True and the corner points in order
    tl = np.array(top_left)
    tr = np.array(top_right)
    br = np.array(bottom_right)
    bl = np.array(bottom_left)
    box = [(tl + tr)/2, (tr + br)/2, (br + bl)/2, (bl + tl)/2]
    # Convert to tuples for output
    box_tuples = [tuple(map(float, point)) for point in box]
    return True, box_tuples

# Example usage (Python 2 style print)
left_line = (float('inf'), 100)   # x=100
right_line = (float('inf'), 200)  # x=200
upper_line = (0, 50)              # y=50
lower_line = (0, 150)             # y=150

is_quad, boundary = check_quadrilateral(left_line, right_line, upper_line, lower_line)
print "Is quadrilateral:", is_quad
print "Boundary points:", boundary











##########################################################

def calculate_midpoints(lines):
    """Calculate midpoints of all input lines"""
    midpoints = []
    for line in lines:
        x1, y1, x2, y2 = line
        mid_x = (x1 + x2) / 2.0
        mid_y = (y1 + y2) / 2.0
        midpoints.append((mid_x, mid_y))
    return midpoints

def is_point_in_rect(point, rect):
    """
    Check if point (x,y) is inside rectangle
    rect: (x_min, y_min, x_max, y_max)
    """
    x, y = point
    x_min, y_min, x_max, y_max = rect
    return (x_min <= x <= x_max) and (y_min <= y <= y_max)

def validate_midpoints(diagonal_U_lines, diagonal_L_lines, rect):
    """
    Validate how many line midpoints are inside rectangle
    Returns: (count_inside, total_points)
    """
    all_lines = diagonal_U_lines + diagonal_L_lines
    midpoints = calculate_midpoints(all_lines)
    
    count = 0
    for point in midpoints:
        if is_point_in_rect(point, rect):
            count += 1
            
    return count, len(midpoints)




##############################
# draw lines
def draw_lines(img, lines):
    global Offset
    for line in lines:
        x1, y1, x2, y2 = line[0]
        color = (random.randint(0, 255), random.randint(0, 255), random.randint(0, 255))
        img = cv2.line(img, (x1, y1+Offset), (x2, y2+Offset), color, 2)
    return img

# draw rectangle
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

# left lines, right lines, diagonal lines, horizental lines
def divide_lines(lines):
    global Width, Height
    global Offset
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
        
        if abs(slope) >= low_slope_threshold and abs(slope) < high_slope_threshold:
            slopes.append(slope)
            new_lines.append(line[0])


    # divide lines left to right
    left_lines = []
    right_lines = []

    # divide lines diagonal and horizental
    diagonal_U_lines = []
    diagonal_L_lines = []
    horizental_U_lines = []
    horizental_L_lines = []

    for j in range(len(slopes)):
        Line = new_lines[j]
        slope = slopes[j]

        x1, y1, x2, y2 = Line
        print(Line)
        if (slope < 0) and (x2 < Width/2 - 90):
            left_lines.append([Line.tolist()])
        elif (slope > 0) and (x1 > Width/2 + 90):
            right_lines.append([Line.tolist()])
        elif (slope > 0.1) and ((x1-Width/2)*(x2-Width/2)<0):
            if (y1<(Height-Offset)/2-10) :
                diagonal_U_lines.append([Line.tolist()])
            elif (y1>(Height-Offset)/2+10) :
                diagonal_L_lines.append([Line.tolist()])           
        elif (slope < 0.1) and (slope > -0.1) and ((x1-Width/2)*(x2-Width/2)<0) :
            if (y1<(Height-Offset)/2-4) :
                diagonal_U_lines.append([Line.tolist()])
            elif (y1>(Height-Offset)/2+4) :
                diagonal_L_lines.append([Line.tolist()])         
    return left_lines, right_lines, diagonal_U_lines, diagonal_L_lines, horizental_U_lines, horizental_L_lines

# get average m, b of lines
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

# get lpos, rpos
def get_line_pos(img, lines, left=False, right=False, diagonal=False):
    global Width, Height
    global Offset, Gap

    m, b = get_line_params(lines)


    if m == 0:
        if left:
            pos = 0
        elif right:
            pos = Width
    else:
        y = Gap / 2
        pos = (y - b) / m

        b += Offset
        x1 = (Height - b) / float(m)
        x2 = ((Height/2) - b) / float(m)

        cv2.line(img, (int(x1), Height), (int(x2), (Height/2)), (255, 0,0), 3)

    return img, int(pos), [m,b]

# show image and return lpos, rpos
def process_image(frame):
    global Width, Height
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
    left_lines, right_lines, diagonal_U_lines, diagonal_L_lines, horizental_U_lines, horizental_L_lines = divide_lines(all_lines)

    # get center of lines
    frame, lpos, left_line = get_line_pos(frame, left_lines, left=True)
    frame, rpos, right_line = get_line_pos(frame, right_lines, right=True)


    # draw a horizental line 
    upper_line = []
    upper_line[1], upper_line[2] = get_line_params(frame, horizental_U_lines)
    if upper_line[1]!= 0.0:
        x1 = int((0.0 - upper_line[2]) / upper_line[1])
        x2 = int((Height - upper_line[2]) / upper_line[1])
        cv2.line(frame, (x1,0), (x2,Height), (255,0,0), 2)
    Upos_horizental = upper_line[1]*Width/2 + upper_line[2]

    # draw a horizental line
    lower_line = []
    lower_line[1], lower_line[2]  = get_line_params(frame, horizental_L_lines)
    if lower_line[1] != 0.0:
        x1 = int((0.0 - lower_line[2]) / lower_line[1])
        x2 = int((Height - lower_line[2]) / lower_line[1])
        cv2.line(frame, (x1,0), (x2,Height), (255,0,0), 2)
    Lpos_horizental = lower_line[1]*Width/2 + lower_line[2]

    is_quad, box = check_quadrilateral(left_line, right_line, upper_line, lower_line)

    if (is_quad):
        count, length = validate_midpoints(diagonal_U_lines,diagonal_L_lines,box)
        print "count:",count
    
    # # draw a diagonal line
    # m_diagonal, b_diagonal = get_line_params(frame, diagonal_U_lines)
    # if m_diagonal != 0.0:
    #     x1 = int((0.0 - b_diagonal) / m_diagonal)
    #     x2 = int((Height - b_diagonal) / m_diagonal)
    #     cv2.line(frame, (x1,0), (x2,Height), (255,0,0), 2)
    # Upos_diagonal = m_diagonal*Width/2 + b_diagonal
    # # draw a diagonal line
    # m_diagonal, b_diagonal = get_line_params(frame, diagonal_L_lines)
    # if m_diagonal != 0.0:
    #     x1 = int((0.0 - b_diagonal) / m_diagonal)
    #     x2 = int((Height - b_diagonal) / m_diagonal)
    #     cv2.line(frame, (x1,0), (x2,Height), (255,0,0), 2)
    # Lpos_diagonal = m_diagonal*Width/2 + b_diagonal


    # horizental 위아래 차선 추출후 좌우위아래 안에 몇개의 라인이 있는지 검사하여 정지선 체커
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


def start():
    global pub
    global image
    global cap
    global Width, Height

    rospy.init_node('auto_drive')
    pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)

    image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, img_callback)
    print "---------- Xycar A2 v1.0 ----------"
    rospy.sleep(2)

    while True:
        while not image.size == (640*480*3):
            continue

        lpos, rpos = process_image(image)

        center = (lpos + rpos) / 2
        angle = PID(center, 0.45, 0.0007, 0.25)
        speed = 5
        drive(angle, speed)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    rospy.spin()

if __name__ == '__main__':
    i_error = 0.0
    prev_error = 0.0
    start_time = time.time()

    start()