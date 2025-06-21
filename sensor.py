#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import cv2
import cv2, math
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, LaserScan
from config import Width, Height,mtx, dist, src_pts, dst_pts, Debug

from line import Line_debug, Line
from collections import deque #stopline_frame_buff 를 위한 큐

class Camera:
    """
    카메라 이미지 처리 클래스:
    - 이미지 보정, 차선 검출, 횡단보도/정지선 인식 기능 수행
    """
    def __init__(self):
        # 이미지 처리 파라미터 초기화
        self.Offset = 340  # 관심 영역(ROI) y축 시작점
        self.Gap = 40       # 관심 영역(ROI) 높이
        
        self.bridge = CvBridge()  # ROS 이미지 ↔ OpenCV 변환기
        # 카메라 캘리브레이션 매트릭스 계산
        self.cal_mtx, self.cal_roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (Width, Height), 1, (Width, Height))

        self.raw_image = np.zeros((480, 640, 3), dtype=np.uint8)
        self.calibration_image = np.zeros((480, 640, 1), dtype=np.uint8)
        self.bird_eye_image = np.zeros((480, 640, 1), dtype=np.uint8)

        # 정지선/수평선 감지 버퍼 (최근 N프레임 결과 저장)
        self.diagonalline_frame_buff = deque([0]*20, maxlen=20)      # 정지선 감지 버퍼
        self.horizentalline_frame_buff = deque([0]*50, maxlen=50) # 수평선 감지 버퍼

        self.crosswalk_completed = False # 횡단보도 통과 상태 플래그

        # 카메라 이미지 구독 설정
        rospy.Subscriber("/usb_cam/image_raw", Image, self.img_callback)
        print("image subscriber start")

        # 디버그 모드에 따른 라인 처리 객체 선택
        if Debug == True:
            self.Line = Line_debug()  # 디버그용 (시각화 포함)
        else:
            self.Line = Line()        # 실전용 (최적화)

    def img_callback(self, data):
        """ROS 이미지 메시지를 OpenCV 이미지로 변환하여 저장"""
        self.raw_image = self.bridge.imgmsg_to_cv2(data, "bgr8")


    def process_calibration_and_birdeye(self):
        """
        이미지 처리 파이프라인:
        1. 이미지 보정
        2. 차선 위치 계산
        3. Bird-Eye 변환
        4. 횡단보도/정지선 인식
        """
        # 1. 이미지 왜곡 보정 (캘리브레이션)
        undistorted = cv2.undistort(self.raw_image, mtx, dist, None, self.cal_mtx)
        x, y, w, h = self.cal_roi
        undistorted = undistorted[y:y + h, x:x + w]
        undistorted = cv2.resize(undistorted, (Width, Height))
        gray = cv2.cvtColor(undistorted, cv2.COLOR_BGR2GRAY)

        # 2. 이미지 전처리 (블러 → 엣지 검출)
        kernel_size = 5
        blur_gray = cv2.GaussianBlur(gray,(kernel_size, kernel_size), 0)


        low_threshold = 60
        high_threshold = 70
        edge_img = cv2.Canny(np.uint8(blur_gray), low_threshold, high_threshold)   # canny edge

        # 3. 차선 검출을 위한 ROI 설정
        roi = edge_img[self.Offset : self.Offset+ self.Gap, 0 : Width]
        self.calibration_image = edge_img
        all_lines = cv2.HoughLinesP(roi,1,math.pi/180,30,30,10)

        # 4. 좌우 차선 위치 계산
        if Debug == True:
            lpos, rpos = self.Line.process_calibration(self.calibration_image, all_lines)
        else:
            lpos, rpos = self.Line.process_calibration(all_lines)

        # 5. Bird-Eye 뷰 변환
        M_perspective = cv2.getPerspectiveTransform(src_pts, dst_pts)
        bird_eye_image = cv2.warpPerspective(gray, M_perspective, (Width, Height))
        self.bird_eye_image = bird_eye_image

        # 6. 횡단보도/대각선/수평선 인식
        is_crosswalk, is_diagonal, is_horizental = self.Line.process_birdeye(bird_eye_image, 
        crosswalk_completed=self.crosswalk_completed)

        # 7. 감지 결과 버퍼 업데이트
        if is_horizental:
            self.horizentalline_frame_buff.append(1)
        else:
            self.horizentalline_frame_buff.append(0)
        if is_diagonal:
            self.diagonalline_frame_buff.append(1)
        else:
            self.diagonalline_frame_buff.append(0)

        # 8. 정지선 판단 로직 (대각선과 수평선을 인식 한 뒤, 대각선과 수평선이 3프레임동안 사라진 경우 정지선으로 판단)
        sum_last3_stopline = sum(list(self.diagonalline_frame_buff)[-3:])
        sum_last3_horizontal = sum(list(self.horizentalline_frame_buff)[-3:])

        # 버퍼 조건 충족 시 정지선 판정
        if (sum(self.diagonalline_frame_buff) >= 5 and
            sum(self.horizentalline_frame_buff) >= 8 and
            sum_last3_stopline == 0 and
            sum_last3_horizontal == 0):
            is_stopline = True
        else: is_stopline = False
        return lpos, rpos, is_crosswalk, is_stopline


class Lidar:
    def __init__(self):
        rospy.Subscriber("/scan", LaserScan, self.lidar_callback)
        print("lidar subscriber start")
        self.cal_mtx, self.cal_roi = cv2.getOptimalNewCameraMatrix(mtx, dist
                                    , (Width, Height), 1, (Width, Height))
        self.lidar_points = None


    def lidar_callback(self, scan):
        self.lidar_points = scan.ranges



    # 장애물 판단 함수( 0.4m 의 전방 기준 100도의 범위를 탐색)
    def is_obstacle_ahead(self, threshold=0.4, check_range=200, count_limit=5):
        countright = 0
        countleft = 0

        #좌/우측 범위 탐색
        for deg in range(check_range+1):
            if 0.01 < self.lidar_points[deg+180] <= threshold:
                countright += 1
            if 0.01 < self.lidar_points[180-deg] <= threshold:
                countleft += 1
        
        #각 case에 맞는 리턴 값 1: 우측 장애물, 2: 좌측 장애물 0: 장애물 없음 
        if countright > count_limit and countleft < count_limit:
            return 1
        elif countright < count_limit and countleft > count_limit:
            return 2
        else:
            return 0

    # 터널 인식 함수 (0.45m 검사, 정 좌/우측 기준 +20도의 범위를 탐색)
    def is_tunnel(self, threshold=0.45, check_range=40, count_limit=5):
        count1 = 0
        count2 = 0
        for deg in range(check_range + 1):
            if np.isinf(self.lidar_points[0]) or np.isinf(self.lidar_points[360]):
                continue
            else:
                if 0.01 < self.lidar_points[0+check_range] <= threshold:
                    count1 += 1
                if 0.01 < self.lidar_points[360-check_range] <= threshold:
                    count2 += 1
        # 두 조건이 모두 만족해야지 터널이라고 판단
        return (count1 > count_limit) and (count2 > count_limit)


    # 우측에 장애물이 있을 경우 주행
    def right_obstacle_driving(self):
        rtn = list()
        distance = 0
        count = 0
        # 라이다 범위를 돌며 0.4m 이내의 라이다 값과 그 각도를 받고, 5개 이상 찍힐 경우 장애물로 판단 후 거리와 각도 리턴
        for i in range(200):
            if not np.isinf(self.lidar_points[i+180]):
                if 0.01 < self.lidar_points[i+180] < 0.4:
                    rtn.append(self.lidar_points[i+180])
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
        # 5개의 점이므로 평균을 내고, 각도 역시 라이다 각도를 현실 각도로 통일
        return distance/5, i/2

    # 좌측에 장애물이 있을 경우 주행
    def left_obstacle_driving(self):
        rtn = list()
        distance = 0
        count = 0
        # 라이다 범위를 돌며 0.4m 이내의 라이다 값과 그 각도를 받고, 5개 이상 찍힐 경우 장애물로 판단 후 거리와 각도 리턴
        for i in range(200):
            if not np.isinf(self.lidar_points[180-i]):
                if 0.01 < self.lidar_points[180-i] < 0.4:
                    rtn.append(self.lidar_points[180-i])
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
        # 5개의 점이므로 평균을 내고, 각도 역시 라이다 각도를 현실 각도로 통일
        return distance/5, i/2

    # 터널 주행 함수
    def tunnel_driving(self):
        left = list()
        right = list()
        # 좌측 우측 라이다 값을 20도 범위만큼 받고, 이 평균 값을 각각 좌측거리, 우측 거리로 사용
        for i in range(40):
            if not np.isinf(self.lidar_points[i]):
                left.append(self.lidar_points[i])
            if not np.isinf(self.lidar_points[360-i]):
                right.append(self.lidar_points[360-i])
        left_distance = sum(left)/len(left)
        right_distance = sum(right)/len(right)
        return left_distance, right_distance

