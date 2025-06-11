#!/usr/bin/env python
# -*- coding: utf-8 -*-
import cv2
import numpy as np
import math
from config import Width, Height, Offset, Gap, Width_Offset

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

def divide_left_right(lines):
    low_thresh, high_thresh = 0.1, 20
    left_lines, right_lines = [], []
    for line in lines:
        x1, y1, x2, y2 = line
        slope = float(y2 - y1) / (x2 - x1 + 1e-5)
        if abs(slope) < low_thresh or abs(slope) > high_thresh:
            continue
        if slope < 0 and x2 < Width // 2 - 90:
            left_lines.append([x1, y1, x2, y2])
        elif slope > 0 and x1 > Width // 2 + 90:
            right_lines.append([x1, y1, x2, y2])
    return left_lines, right_lines

def get_line_params(lines):
    if len(lines) == 0:
        return 0, 0
    x_sum, y_sum, m_sum = 0.0, 0.0, 0.0
    for x1, y1, x2, y2 in lines:
        x_sum += x1 + x2
        y_sum += y1 + y2
        m_sum += float(y2 - y1) / (x2 - x1 + 1e-5)
    x_avg = x_sum / (2 * len(lines))
    y_avg = y_sum / (2 * len(lines))
    m = m_sum / len(lines)
    b = y_avg - m * x_avg
    return m, b

def get_line_pos(img, lines, left=False, right=False):
    m, b = get_line_params(lines)
    if m == 0 and b == 0:
        return img, 0 if left else Width
    y = Gap / 2
    pos = int((y - b) / m)
    return img, pos

def classify_line_region_and_lane(frame):
    blur = cv2.GaussianBlur(frame, (5, 5), 0)
    edge = cv2.Canny(blur, 60, 70)
    roi = edge[Offset : Offset+Gap, 0 : Width]
    lines = cv2.HoughLinesP(roi, 1, math.pi / 180, 30, minLineLength=30, maxLineGap=10)

    result = "none"
    lpos, rpos = 0, Width
    left_lines = None
    right_lines = None

    if lines is not None:
        lines_reshaped = [[x1 + Width_Offset, y1 + Offset, x2 + Width_Offset, y2 + Offset]
                        for [[x1, y1, x2, y2]] in lines]
        horizontal_count = count_lines_by_slope(lines, 0.0, 0.1)
        diagonal_count = count_lines_by_slope(lines, 0.3, 1.0)
        vertical_count = count_lines_by_slope(lines, 2.0, float('inf'))

        if horizontal_count >= 1:
            if vertical_count >= 6:
                result = "crosswalk"
            elif diagonal_count >= 3:
                result = "stop_line"
            else:
                result = "start_line"

        left_lines, right_lines = divide_left_right(lines)

        frame, lpos = get_line_pos(frame, left_lines, left=True)
        frame, rpos = get_line_pos(frame, right_lines, right=True)
        frame = cv2.line(frame, (230, 235), (410, 235), (255,255,255), 2)

            # show image
        cv2.imshow('calibration', frame)

    return result, lpos, rpos, left_lines, right_lines


def plot_lane_detection(frame, result, lpos, rpos, left_lines, right_lines, lidar_mask=None, show_center=True, window_name="Lane + LiDAR Visualization"):
    # frame: 입력 gray 이미지 (bird-eye view)
    # result: 검출 결과 문자열
    # lpos, rpos: 좌/우 차선 x좌표
    # left_lines, right_lines: 검출된 좌/우 차선 리스트
    # lidar_mask: LiDAR 마스크 (0/1) 이미지 (optional)
    # show_center: 중심점 시각화 여부

    debug_img = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
    if(left_lines!=None and right_lines!=None):
        # 좌/우 차선 시각화
        for x1, y1, x2, y2 in left_lines:
            cv2.line(debug_img, (x1, y1), (x2, y2), (255,0,0), 2)
        for x1, y1, x2, y2 in right_lines:
            cv2.line(debug_img, (x1, y1), (x2, y2), (0,255,0), 2)
        # 좌/우 차선 중심점
        cv2.rectangle(debug_img, (lpos-5, 15+Offset), (lpos+5, 25+Offset), (0,255,0), 2)
        cv2.rectangle(debug_img, (rpos-5, 15+Offset), (rpos+5, 25+Offset), (0,255,0), 2)
        # 중앙점
        if show_center:
            center = (lpos + rpos) // 2
            cv2.rectangle(debug_img, (center-5, 15+Offset), (center+5, 25+Offset), (0,255,0), 2)
            cv2.rectangle(debug_img, (315, 15+Offset), (325, 25+Offset), (0,0,255), 2)
    # 검출 결과 텍스트
    cv2.putText(debug_img, "Detected: {}".format(result), (30, 40),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)

    # LiDAR 마스크 오버레이 (빨간색)
    if lidar_mask is not None:
        lidar_overlay = (lidar_mask * 255).astype(np.uint8)
        lidar_color = cv2.cvtColor(lidar_overlay, cv2.COLOR_GRAY2BGR)
        lidar_color[:, :, 0] = 0  # Blue 채널 0
        lidar_color[:, :, 1] = 0  # Green 채널 0
        # 빨간색만 남기고 오버레이
        debug_img = cv2.addWeighted(debug_img, 0.7, lidar_color, 0.3, 0)


    cv2.imshow(window_name, debug_img)
    cv2.waitKey(1)