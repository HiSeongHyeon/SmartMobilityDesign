#!/usr/bin/env python
# -*- coding: utf-8 -*-
import cv2
import numpy as np
import math, random

from config import Width, Height, Offset, Gap, Width_Offset
from config import bird_eye_roi_x_start, bird_eye_roi_x_end, bird_eye_roi_y_start, bird_eye_roi_y_end

class Line_debug:
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
        # 하이퍼 파라미터 
        # |slope| < 0.1 혹은 |slope| > 20인 경우는 건너뜁니다.
        low_thresh, high_thresh = 0.1, 20 
        left_lines, right_lines = [], []
        for line in lines:
            x1, y1, x2, y2 = line[0]
            slope = float(y2 - y1) / (x2 - x1 + 1e-5)
            if abs(slope) < low_thresh or abs(slope) > high_thresh:
                continue
            # 기울기가 음수이고, x2가 이미지 좌측에 있으면 왼쪽 차선 후보로 판단
            if slope < 0 and x2 < Width // 2 - 90:
                left_lines.append([line[0]])
            # 기울기가 양수이고, x1이 이미지 우측에 있으면 오른쪽 차선 후보로 판단
            elif slope > 0 and x1 > Width // 2 + 90:
                right_lines.append([line[0]])
        return left_lines, right_lines


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
    def get_line_pos(self, img, lines, left=False, right=False):
        global Width, Height
        global Offset, Gap

        m, b = self.get_line_params(lines)

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

    # def classify_line_region_and_lane(frame):
    #     print " classify 1"
    #     blur = cv2.GaussianBlur(frame, (5, 5), 0)
    #     edge = cv2.Canny(blur, 60, 70)
    #     roi = edge[Offset : Offset+Gap, 0 : Width]
    #     lines = cv2.HoughLinesP(roi, 1, math.pi / 180, 30, minLineLength=30, maxLineGap=10)

    #     result = "none"
    #     lpos, rpos = 0, Width
    #     left_lines = None
    #     right_lines = None
    #     print " classify 2"
    #     if lines is not None:
    #         # lines_reshaped = [[x1 + Width_Offset, y1 + Offset, x2 + Width_Offset, y2 + Offset]
    #         #                 for [[x1, y1, x2, y2]] in lines]
    #         # horizontal_count = count_lines_by_slope(lines, 0.0, 0.1)
    #         # diagonal_count = count_lines_by_slope(lines, 0.3, 1.0)
    #         # vertical_count = count_lines_by_slope(lines, 2.0, float('inf'))
    #         # print " classify 3"
    #         # if horizontal_count >= 1:
    #         #     if vertical_count >= 6:
    #         #         result = "crosswalk"
    #         #     elif diagonal_count >= 3:
    #         #         result = "stop_line"
    #         #     else:
    #         #         result = "start_line"
    #         print " classify 4"
    #         left_lines, right_lines = divide_left_right(lines)
    #         print " classify 5"
    #         frame, lpos = get_line_pos(frame, left_lines, left=True)
    #         frame, rpos = get_line_pos(frame, right_lines, right=True)
    #         frame = cv2.line(frame, (230, 235), (410, 235), (255,255,255), 2)
    #         print " classify 6"
    #             # show image
    #         cv2.imshow('calibration', frame)
    #     print " classify 7"
    #     return result, lpos, rpos, left_lines, right_lines

    
    # show image and return lpos, rpos
    def process_calibration(self, frame, all_lines):

        # divide left, right lines
        if all_lines is None:
            return 0, 640
        left_lines, right_lines = self.divide_left_right(all_lines)

        # get center of lines
        frame, lpos = self.get_line_pos(frame, left_lines, left=True)
        frame, rpos = self.get_line_pos(frame, right_lines, right=True)

        # draw lines
        frame = self.draw_lines(frame, left_lines)
        frame = self.draw_lines(frame, right_lines)
        frame = cv2.line(frame, (230, 235), (410, 235), (255,255,255), 2)
                                    
        # draw rectangle
        frame = self.draw_rectangle(frame, lpos, rpos, offset=Offset)
        #roi2 = cv2.cvtColor(roi, cv2.COLOR_GRAY2BGR)
        #roi2 = draw_rectangle(roi2, lpos, rpos)

        # show image
        cv2.imshow('calibration', frame)

        return lpos, rpos


        # 횡단보도인지?
    def process_birdeye(bird_eye_frame):
        """
        Bird-Eye View 프레임에서 ROI 영역 내 수직선 개수를 바탕으로
        횡단보도인지 여부를 판단 (기울기를 각도로 변환하여 기준 적용)
        """
        bird_eye_roi = bird_eye_frame[bird_eye_roi_y_start:bird_eye_roi_y_end, bird_eye_roi_x_start:bird_eye_roi_x_end]

        blur = cv2.GaussianBlur(bird_eye_roi, (5, 5), 0)
        edge = cv2.Canny(blur, 70, 90)

        # HoughLinesP로 직선 검출
        # 20개 이상 누적되면 선분으로 판단 // 최소 길이 5픽셀 이상 // 간격이 10픽셀 이하일 경우 하나의 선분으로 간주
        lines = cv2.HoughLinesP(edge, 1, math.pi / 180, threshold=20,
                                minLineLength=10, maxLineGap=10)
        vertical_count = 0

        color_frame = cv2.cvtColor(bird_eye_frame, cv2.COLOR_GRAY2BGR)

        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                dx = x2 - x1
                dy = y2 - y1

                if dx == 0:
                    angle_deg = 90.0
                else:
                    slope = dy / dx
                    angle_deg = abs(math.degrees(math.atan(slope)))

                if angle_deg >= 80.0:  # 각도로 수직선 판단 (75도 이상)
                    vertical_count += 1

                    # 수직선 시각화
                    cv2.line(color_frame,
                            (x1 + bird_eye_roi_x_start, y1 + bird_eye_roi_y_start),
                            (x2 + bird_eye_roi_x_start, y2 + bird_eye_roi_y_start),
                            (0, 0, 255), 2)



        # 수직선 개수 출력
        text = "Vertical lines: {}".format(vertical_count)
        cv2.putText(color_frame, text, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

        # ROI 영역 시각화 박스 추가
        cv2.rectangle(color_frame, (bird_eye_roi_x_start, bird_eye_roi_y_start),
                (bird_eye_roi_x_end, bird_eye_roi_y_end), 255, 2)


        cv2.imshow("Birdeye", color_frame)
        # 수직선이 1개 이상이면 횡단보도
        return vertical_count >= 1


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