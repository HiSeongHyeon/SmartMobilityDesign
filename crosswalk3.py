#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
고려되는 부분
1. [2개의 frame을 사용] Tracking은 기존 카메라 // 횡단보도, 정지선 인식은 Bird-Eye View 사용

<해결해야 하는 부분>
1. 정지선, 횡단보도 인식을 위한 parameter 값 tuning
2. 차선 인식 ROI 설정
3. 

<효본이형네>
1. Bird Eye View 안쓸 때 -> 0.2 ~ 0.5 : 대각선 // 0.7 이상 수직선
2. 횡단보도의 경우, 횡단보도용 roi를 설정했고, 수직선 1개가 여러개로 인식되는 문제 발생 -> 1개만 인식되어도 횡단보도라고 판단하도록 코드 변경!
"""

import rospy
import numpy as np
import cv2, random, math, time
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String
from xycar_msgs.msg import xycar_motor

# ==========================
# Bird-eye view 설정 (src/dst 좌표)
# ==========================

# 실제 도로 영역. 가까이 있는 차선은 넓게, 멀리 있는 차선은 좁은 형태
src_pts = np.float32([[200, 300], # 좌측 상단 (왼쪽 차선 위)
                    [440, 300], # 우측 상단 (오른쪽 차선 위)
                    [100, 480], # 좌측 하단 (왼쪽 차선 아래)
                    [540, 480]] # 우측 하단 (오른쪽 차선 아래)
                    )
                    
# 네 점은 위에서 내려다본 직사각형 형태의 도로를 표현
dst_pts = np.float32([[200, 0],
                    [440, 0],
                    [200, 480],
                    [440, 480]])
M_perspective = cv2.getPerspectiveTransform(src_pts, dst_pts)


# ==========================
# 전역 변수 설정
# ==========================
DEBUG = False

raw_image = np.empty(shape=[0])
calibration_image = np.empty(shape=[0])
bird_eye_image = np.empty(shape=[0])


bridge = CvBridge()
pub = None
info_pub = None
Width = 640
Height = 480

# Bird eye view의 경우, 조금 더 멀리 봐야 함.
# 가까워지면 직선이 왜곡됨.
Offset = 100 # 영상 프레임의 상단으로부터 몇 픽셀 아래부터 시작할지를 지정
Gap = 40 # ROI의 세로 높이를 의미
Width_Offset = 40

i_error = 0.0
prev_error = 0.0
start_time = time.time()

crosswalk_detected = False
stop_completed = False
horizontal_line_detected = False


# ==========================
# 카메라 보정 파라미터
# ==========================
mtx = np.array([[340.876013, 0.000000, 333.212353],
                [0.000000, 341.790625, 241.433953],
                [0.000000, 0.000000, 1.000000]])
dist = np.array([-0.302220, 0.069858, 0.000204, -0.002379, 0.000000])
cal_mtx, cal_roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (Width, Height), 1, (Width, Height))

# ==========================
# 유틸리티 함수
# ==========================

# def get_left_right_extreme_positions_with_split(lines):
#     """
#     각 차선을 2개의 수직선으로 구성된 경계선 쌍으로 간주하여
#     좌우 차선을 나누고, 각 차선의 대표적인 x 위치를 반환.
#     추가로 좌우 차선 선분도 반환.
#     """
#     low_thresh, high_thresh = 0.1, 20
#     left_lines, right_lines = [], []

#     for x1, y1, x2, y2 in lines:
#         slope = float(y2 - y1) / (x2 - x1 + 1e-5)
#         if abs(slope) < low_thresh or abs(slope) > high_thresh:
#             continue
#         if slope < 0 and x2 < Width // 2 - 90:
#             left_lines.append([x1, y1, x2, y2])
#         elif slope > 0 and x1 > Width // 2 + 90:
#             right_lines.append([x1, y1, x2, y2])

#     def get_center_x(line_group):
#         if len(line_group) == 0:
#             return 0
#         x_positions = [x1 for x1, _, x2, _ in line_group] + [x2 for x1, _, x2, _ in line_group]
#         return int(sum(x_positions) / len(x_positions))

#     lpos = get_center_x(left_lines)
#     rpos = get_center_x(right_lines) if right_lines else Width

#     # ➜ 선 리스트도 함께 반환
#     return lpos, rpos, left_lines, right_lines


def draw_lines(img, lines):
    global Offset
    for line in lines:
        x1, y1, x2, y2 = line[0]
        if x2 - x1 == 0:
            slope = float('inf')
        else:
            slope = float(y2 - y1) / float(x2 - x1)

        abs_slope = abs(slope)
        if abs_slope <= 0.1:
            color = (0, 255, 0)  # horizontal - green
        elif 0.3 <= abs_slope <= 1.0:
            color = (255, 0, 255)  # diagonal - purple
        elif abs_slope >= 2.0:
            color = (0, 0, 255)  # vertical - red
        else:
            color = (0, 0, 0)  # 기타 - 검정

        cv2.line(img, (x1, y1 + Offset), (x2, y2 + Offset), color, 2)
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
    global raw_image, calibration_image, bird_eye_image
    # 만약 global이 없으면, 그 변수는 함수 내에서 지역 변수로 새롭게 생성
    raw_image = bridge.imgmsg_to_cv2(data, "bgr8")

    # 1. 카메라 왜곡 보정
    tf_image = cv2.undistort(raw_image, mtx, dist, None, cal_mtx)
    x, y, w, h = cal_roi
    tf_image = tf_image[y:y + h, x:x + w]

    # gray scale
    gray_image = cv2.cvtColor(tf_image, cv2.COLOR_BGR2GRAY)

    # 2. 원본 프레임 (리사이즈 + 보정만 된)
    calibration_image = cv2.resize(gray_image, (Width, Height))

    # 4. BEV 변환
    bird_eye_image = cv2.warpPerspective(calibration_image, M_perspective, (Width, Height))

    # 5. 시각화
    if(DEBUG == True):
        cv2.imshow("Raw View", raw_image)
    cv2.imshow("Calibration View", calibration_image)
    cv2.imshow("Bird Eye View", bird_eye_image)
    cv2.waitKey(1)

    # return value는 자동으로 버려짐
    # return calibration_image, bird_eye_image

# **주어진 직선들의 기울기(slope)**를 계산하고, 
# 기울기의 절댓값이 특정 범위 (low ~ high)에 속하는 직선의 개수를 세는 함수입니다.
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

# 기존(pid_hough_drive.py)의 lpos, rpos
def divide_left_right_original(lines):
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

    return left_lines, right_lines

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

def get_line_params(lines):
    if len(lines) == 0:
        return 0, 0
    x_sum, y_sum, m_sum = 0.0, 0.0, 0.0
    for line in lines:
        x1, y1, x2, y2 = line[0]
        x_sum += x1 + x2
        y_sum += y1 + y2
        m_sum += float(y2 - y1) / (x2 - x1 + 1e-5)

    # 모든 직선의 중간점을 평균 내어 대표 점 (xavg,yavg)(xavg​,yavg​) 계산
    x_avg = x_sum / (2 * len(lines))
    y_avg = y_sum / (2 * len(lines))
    m = m_sum / len(lines)
    b = y_avg - m * x_avg

    # 직선 방정식 y=mx+b 형태에서 m과 b를 의미
    return m, b

# 주어진 직선 그룹의 평균적인 위치(좌우 차선 위치)를 x 좌표로 계산하는 역할
# 주로 차선 중심 위치를 계산하여 PID 제어의 기준점으로 사용
def get_line_pos(img, lines, left=False, right=False):
    m, b = get_line_params(lines)
    if m == 0 and b == 0:
        return img, 0 if left else Width
    y = Gap / 2
    pos = int((y - b) / m)
    return img, pos

# 횡단보도인지?
def is_crosswalk(bird_eye_frame):
    """
    Bird-Eye View 프레임에서 ROI 영역 내 수직선 개수를 바탕으로
    횡단보도인지 여부를 판단
    """
    # ROI 설정
    roi_x_start = 200
    roi_x_end = 500
    roi_y_start = 100
    roi_y_end = 140
    roi = bird_eye_frame[roi_x_start:roi_x_end, roi_y_start:roi_y_end]

    # 전처리: 그레이스케일 → 블러 → 캐니 엣지
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    edge = cv2.Canny(blur, 60, 70)

    # HoughLinesP로 직선 검출
    lines = cv2.HoughLinesP(edge, 1, math.pi / 180, threshold=30,
                            minLineLength=30, maxLineGap=10)

    # 수직선 개수 세기
    vertical_count = 0
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            if x2 - x1 == 0:
                slope = float('inf')
            else:
                slope = float(y2 - y1) / (x2 - x1)
            if abs(slope) >= 2.0:  # 수직선 기준
                vertical_count += 1

    # 기준 이상이면 횡단보도로 판단
    return vertical_count >= 1  # 수직선 4개 이상이면 횡단보도

# show image and return lpos, rpos
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
    left_lines, right_lines = divide_left_right_original(all_lines)

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

    return lpos, rpos

def classify_line_region_and_lane(frame):
    global crosswalk_detected, stop_completed, horizontal_line_detected

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    """
    high threshold (70): 강한 edge로 직접 판정되는 픽셀 기준
    low threshold (60): 약한 edge 후보로 남길 수 있는 최소값, 이 값보다 작은 그래디언트는 무시
    
    두 값의 간격이 너무 좁으면 edge 누락 위험
    간격이 너무 넓으면 노이즈에 민감
    """
    edge = cv2.Canny(blur, 60, 70)
    roi = edge[Offset:Offset + Gap, Width_Offset:Width-Width_Offset]
    lines = cv2.HoughLinesP(roi, 1, math.pi / 180, 30, minLineLength=30, maxLineGap=10)

    result = "none"
    lpos, rpos = 0, Width

    if lines is not None:
        lines = [line[0] for line in lines]
        lines_reshaped = [[x1, y1, x2, y2] for x1, y1, x2, y2 in lines]

        horizontal_count = count_lines_by_slope(lines_reshaped, 0.0, 0.1)
        diagonal_count = count_lines_by_slope(lines_reshaped, 0.3, 1.0)
        vertical_count = count_lines_by_slope(lines_reshaped, 2.0, float('inf'))

        horizontal_line_detected = horizontal_count >= 1

        if horizontal_line_detected:
            # 횡단보도
            if vertical_count >= 6:
                crosswalk_detected = True
                result = "crosswalk"
            # 정지선
            elif diagonal_count >= 3:
                result = "stop_line"
            # 시작선
            else:
                result = "start_line"

        # # ROI 끝
        # lpos, rpos = get_left_right_extreme_positions_with_split(lines_reshaped)

        # ROI
        left_lines, right_lines = divide_left_right([[line] for line in lines_reshaped])
        _, lpos = get_line_pos(frame, left_lines, left=True)
        _, rpos = get_line_pos(frame, right_lines, right=True)

        debug_img = frame.copy()
        draw_lines(debug_img, left_lines)
        draw_lines(debug_img, right_lines)
        draw_rectangle(debug_img, lpos, rpos, offset=Offset)
        cv2.putText(debug_img, "Detected: {}".format(result), (30, 40), cv2.FONT_HERSHEY_SIMPLEX,
                    1, (0, 255, 255), 2)
        cv2.imshow("Lane Visualization", debug_img)

    return result, lpos, rpos

def start():
    global pub, info_pub, raw_image, calibration_image, bird_eye_image, crosswalk_detected, stop_completed
    rospy.init_node('auto_drive')
    pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
    info_pub = rospy.Publisher('xycar_info', String, queue_size=1)

    # 여기서 img_callback
    rospy.Subscriber("/usb_cam/image_raw", Image, img_callback)
    print("---------- Xycar A2 start ----------")
    rospy.sleep(2)

    while not rospy.is_shutdown():
        if calibration_image.size != (640 * 480 * 3):
            continue

        # 수정 1. 여기서 기존 프레임의 lpos, rpos 차선 인식으로 수행해야 함!
        lpos, rpos = process_image(calibration_image)
        center = int((lpos + rpos) / 2.0)

        # [횡단보도] 횡단보도 체크 함수!!
        if(is_crosswalk(bird_eye_image) and not stop_completed):
            drive(0, 0)
            time.sleep(5)
            stop_completed = True
        # elif result == "stop_line":
        #     print("stop line is detected")
        # elif result == "start_line":
        #     print("start line is detected")

        angle = PID(center, 0.45, 0.0007, 0.25)
        drive(angle, 5)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

if __name__ == '__main__':
    start()