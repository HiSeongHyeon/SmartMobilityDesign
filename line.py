#!/usr/bin/env python
# -*- coding: utf-8 -*-
import cv2
import math, random

from config import Width, Height, Offset, Gap

class Line_debug:

    def __init__(self):
        # config에서 bird-eye ROI 영역 좌표값을 가져와서 인스턴스 변수로 저장
        from config import bird_eye_roi_x_start, bird_eye_roi_x_end, bird_eye_roi_y_start, bird_eye_roi_y_end
        self.bird_eye_roi_x_start = bird_eye_roi_x_start
        self.bird_eye_roi_x_end = bird_eye_roi_x_end
        self.bird_eye_roi_y_start = bird_eye_roi_y_start
        self.bird_eye_roi_y_end = bird_eye_roi_y_end


    @staticmethod
    def draw_lines(img, lines):
        # 입력된 직선 리스트를 이미지에 랜덤 색상으로 그려줌
        for line in lines:
            x1, y1, x2, y2 = line[0]
            color = (random.randint(0, 255), random.randint(0, 255), random.randint(0, 255))
            img = cv2.line(img, (x1, y1+Offset), (x2, y2+Offset), color, 2)
        return img

    @staticmethod
    def draw_rectangle(img, lpos, rpos, offset=0):
        #좌측, 우측, 중앙, 기준선 위치에 사각형을 그려 시각화
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



    @staticmethod
    def divide_left_right(lines):
        # 직선의 기울기를 계산해 왼쪽/오른쪽 라인으로 분류
        # 하이퍼 파라미터
        # |slope| < 0.1 혹은 |slope| > 20인 경우는 건너뜁니다.

        low_slope_threshold = 0.001
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

            if (slope < 0) and (x2 < Width/2 + 25):
                left_lines.append([Line.tolist()])
            elif (slope > 0) and (x1 > Width/2 - 25):
                right_lines.append([Line.tolist()])

        return left_lines, right_lines

    @staticmethod
    def get_line_params(lines):
        #  여러 직선의 평균 기울기(m)와 절편(b) 계산
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

    # get lpos or rpos
    def get_line_pos(self, img, lines, left=False, right=False):
        # 계산된 기울기와 절편으로 라인의 위치(pos) 계산, 이미지에 선 시각화
        m, b = self.get_line_params(lines)
        if abs(m) <0.05 and b == 0:
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

    def process_calibration(self, frame, all_lines):
        """
        Calibration 프레임의 전체 라인에서 좌/우 라인 분리
        lpos, rpos 계산, Calibration 프레임에 시각화 후 lpos, rpos 반환
        """
        

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


        return lpos, rpos



    def process_birdeye(self, bird_eye_frame, crosswalk_completed):
        """
        Bird-Eye View 프레임에서 ROI 내 검출된 선분의 각도를 분석하여
        수직선, 대각선, 수평선 검출한 뒤 Bird-Eye View 프레임과 함께 시각화 함
        수직선 개수로 횡단보도인지 여부를 판단 및 반환함,
        또한 정지선 검출을 위해 대각선과 수평선 검출 여부를 반환함
        """
        # ROI 좌표 설정 (crosswalk_completed 여부에 따라 영역 조정)
        bird_eye_roi_x_start = self.bird_eye_roi_x_start
        bird_eye_roi_x_end = self.bird_eye_roi_x_end
        bird_eye_roi_y_start = self.bird_eye_roi_y_start
        bird_eye_roi_y_end = self.bird_eye_roi_y_end
        if(crosswalk_completed):
            bird_eye_roi_y_start = 00# 280
            bird_eye_roi_y_end = 120 #440
        bird_eye_roi = bird_eye_frame[bird_eye_roi_y_start:bird_eye_roi_y_end, bird_eye_roi_x_start:bird_eye_roi_x_end]
        blur = cv2.GaussianBlur(bird_eye_roi, (5, 5), 0)
        edge = cv2.Canny(blur, 70, 90)

        # HoughLinesP로 직선 검출
        # 20개 이상 누적되면 선분으로 판단 // 최소 길이 5픽셀 이상 // 간격이 10픽셀 이하일 경우 하나의 선분으로 간주
        lines = cv2.HoughLinesP(edge, 1, math.pi / 180, threshold=40,
                                minLineLength=10, maxLineGap=10)
        vertical_count = 0    # 수직선 개수
        diagonal_count = 0    # 대각선 개수
        horizental_count = 0  # 수평선 개수
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
                    angle_deg = math.degrees(math.atan(slope))

                if angle_deg <= 10.0 and angle_deg >= -40:  # 수평선 기준
                    horizental_count += 1
                    # 수평선 시각화
                    cv2.line(color_frame,
                        (x1 + bird_eye_roi_x_start, y1 + bird_eye_roi_y_start),
                        (x2 + bird_eye_roi_x_start, y2 + bird_eye_roi_y_start),
                        (0, 0, 255), 2)
                if angle_deg <= 60.0 and angle_deg >= 0:  # 대각선 기준
                    diagonal_count += 1
                    # 대각선 시각화
                    cv2.line(color_frame,
                        (x1 + bird_eye_roi_x_start, y1 + bird_eye_roi_y_start),
                        (x2 + bird_eye_roi_x_start, y2 + bird_eye_roi_y_start),
                        (0, 0, 255), 2)
                elif abs(angle_deg) >= 80.0:  # 각도로 수직선 판단 (80도 이상)
                    vertical_count += 1

                    # 수직선 시각화
                    cv2.line(color_frame,
                            (x1 + bird_eye_roi_x_start, y1 + bird_eye_roi_y_start),
                            (x2 + bird_eye_roi_x_start, y2 + bird_eye_roi_y_start),
                            (0, 0, 255), 2)

        #대각선 개수, 수직선 개수, 수평선 개수 시각화
        text_diagonal = "Diagonal lines: {}".format(diagonal_count)
        text_vertical = "Vertical lines: {}".format(vertical_count)
        text_horizental = "Horizental lines: {}".format(horizental_count)
        cv2.putText(color_frame, text_diagonal, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        cv2.putText(color_frame, text_horizental, (10, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        cv2.putText(color_frame, text_vertical, (10, 70),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        # ROI 영역 시각화 박스 추가
        cv2.rectangle(color_frame, (bird_eye_roi_x_start, bird_eye_roi_y_start),
                (bird_eye_roi_x_end, bird_eye_roi_y_end), 255, 2)


        cv2.imshow("Birdeye", color_frame)
        # 수직선이 9개 이상이면 is_crosswalk, 대각선 2개 이상이면 is_diagonal, 수평선 2개 이상이면 is_horizental
        return vertical_count >= 9, diagonal_count >= 2, horizental_count >=3


class Line(Line_debug):
    """
    Line_debug에서 시각화 관련 기능을 제거한 실질적 차선 검출 및 횡단보도 판단 및 정지선 검출 클래스
    """
    @staticmethod
    def divide_left_right(lines):
        # 하이퍼 파라미터
        # |slope| < 0.1 혹은 |slope| > 20인 경우는 건너뜁니다.

        low_slope_threshold = 0.001
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

            if (slope < 0) and (x2 < Width/2 + 25):
                left_lines.append([Line.tolist()])
            elif (slope > 0) and (x1 > Width/2 - 25):
                right_lines.append([Line.tolist()])

        return left_lines, right_lines



    # get lpos, rpos
    def get_line_pos(self, lines, left=False, right=False):
        m, b = self.get_line_params(lines)
        if abs(m) <0.05 and b == 0:
            if left:
                pos = 0
            if right:
                pos = Width
        else:
            y = Gap / 2
            pos = (y - b) / m


        return int(pos)

    # show image and return lpos, rpos
    def process_calibration(self, all_lines):
        """
        Line_debug에서는 시각화를 위해 frame을 인자로 받았는데, 해당 함수 인자에는 frame이 삭제됨.
        Calibration 프레임의 전체 라인에서 좌/우 라인 분리
        lpos, rpos 계산, lpos, rpos 반환
        """
        # divide left, right lines
        if all_lines is None:
            return 0, 640
        left_lines, right_lines = self.divide_left_right(all_lines)

        # get center of lines
        lpos = self.get_line_pos(left_lines, left=True)
        rpos = self.get_line_pos(right_lines, right=True)


        return lpos, rpos


        # 횡단보도인지?
    def process_birdeye(self, bird_eye_frame, crosswalk_completed):
        """
        Bird-Eye View 프레임에서 ROI 내 검출된 선분의 각도를 분석하여
        수직선, 대각선, 수평선 검출함.
        수직선 개수로 횡단보도인지 여부를 판단 및 반환함,
        또한 정지선 검출을 위해 대각선과 수평선 검출 여부를 반환함
        """
        # ROI 좌표 설정 (crosswalk_completed 여부에 따라 영역 조정)
        bird_eye_roi_x_start = self.bird_eye_roi_x_start
        bird_eye_roi_x_end = self.bird_eye_roi_x_end
        bird_eye_roi_y_start = self.bird_eye_roi_y_start
        bird_eye_roi_y_end = self.bird_eye_roi_y_end
        if(crosswalk_completed):
            bird_eye_roi_y_start = 0
            bird_eye_roi_y_end = 120 
        bird_eye_roi = bird_eye_frame[bird_eye_roi_y_start:bird_eye_roi_y_end, bird_eye_roi_x_start:bird_eye_roi_x_end]
        blur = cv2.GaussianBlur(bird_eye_roi, (5, 5), 0)
        edge = cv2.Canny(blur, 70, 90)
        # HoughLinesP로 직선 검출
        # 20개 이상 누적되면 선분으로 판단 // 최소 길이 5픽셀 이상 // 간격이 10픽셀 이하일 경우 하나의 선분으로 간주
        lines = cv2.HoughLinesP(edge, 1, math.pi / 180, threshold=40,
                                minLineLength=10, maxLineGap=10)
        vertical_count = 0    # 수직선 개수
        diagonal_count = 0    # 대각선 개수
        horizental_count = 0  # 수평선 개수

        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                dx = x2 - x1
                dy = y2 - y1

                if dx == 0:
                    angle_deg = 90.0
                else:
                    slope = dy / dx
                    angle_deg = math.degrees(math.atan(slope))

                if angle_deg <= 10.0 and angle_deg >= -40:  # 대각선 기준
                    horizental_count += 1

                if angle_deg <= 60.0 and angle_deg >= 0:  # 대각선 기준
                    diagonal_count += 1


                elif abs(angle_deg) >= 80.0:  # 각도로 수직선 판단 (80도 이상)
                    vertical_count += 1


        # 수직선이 1개 이상이면 횡단보도
        return vertical_count >= 9, diagonal_count >= 2, horizental_count >=3

