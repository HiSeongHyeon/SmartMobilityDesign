#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, time
import numpy as np
import cv2
import cv2, math
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import String
from config import Width, Height, R_lidar2cam, T_lidar2cam, mtx, dist, src_pts, dst_pts, Debug

from line import Line_debug
from collections import deque #stopline_frame_buff 를 위한 큐

class Camera:
    def __init__(self):
        self.Offset = 340
        self.Gap = 40

        self.bridge = CvBridge()    
        self.cal_mtx, self.cal_roi = cv2.getOptimalNewCameraMatrix(mtx, dist
                                    , (Width, Height), 1, (Width, Height))
        self.raw_image = np.zeros((480, 640, 3), dtype=np.uint8)
        self.calibration_image = np.zeros((480, 640, 1), dtype=np.uint8)
        self.bird_eye_image = np.zeros((480, 640, 1), dtype=np.uint8)
    
        # stopline_frame_buff: 최근 10프레임의 stopline 감지 결과 저장
        self.stopline_frame_buff = deque([0]*7, maxlen=7)
        rospy.Subscriber("/usb_cam/image_raw", Image, self.img_callback)
        print("image subscriber start") 
        if Debug == True:
            self.Line = Line_debug()


    def img_callback(self, data):
        self.raw_image = self.bridge.imgmsg_to_cv2(data, "bgr8")


    def process_calibration_and_birdeye(self):
        undistorted = cv2.undistort(self.raw_image, mtx, dist, None, self.cal_mtx)
        x, y, w, h = self.cal_roi
        undistorted = undistorted[y:y + h, x:x + w]
        undistorted = cv2.resize(undistorted, (Width, Height))
        gray = cv2.cvtColor(undistorted, cv2.COLOR_BGR2GRAY)

        # blur
        kernel_size = 5
        blur_gray = cv2.GaussianBlur(gray,(kernel_size, kernel_size), 0)

        # canny edge
        low_threshold = 60
        high_threshold = 70
        edge_img = cv2.Canny(np.uint8(blur_gray), low_threshold, high_threshold)


        roi = edge_img[self.Offset : self.Offset+ self.Gap, 0 : Width]

        self.calibration_image = edge_img
        all_lines = cv2.HoughLinesP(roi,1,math.pi/180,30,30,10)
        if Debug == True:
            lpos, rpos = self.Line.process_calibration(self.calibration_image, all_lines)

        M_perspective = cv2.getPerspectiveTransform(src_pts, dst_pts)
        bird_eye_image = cv2.warpPerspective(gray, M_perspective, (Width, Height))
        self.bird_eye_image = bird_eye_image
        if Debug == True:
            is_crosswalk, is_diagonal = self.Line.process_birdeye(bird_eye_image)
        if is_diagonal:
            self.stopline_frame_buff.append(1)
        else:
            self.stopline_frame_buff.append(0)
        if sum(self.stopline_frame_buff) >= 3:
            is_stopline = True
        else: is_stopline = False
        return lpos, rpos, is_crosswalk, is_stopline

    # def change_brid_eye(self, data):
    #     print"b"
    #     bird_eye_gray, gray = self.undistort_and_birdseye(data, self.cal_mtx, self.cal_roi)
    #     if (bird_eye_gray is None or gray is None):
    #         print("error")
    #     print"c"
    #     # if control.lidar_mask is None:
    #     #     return
    #     # final_image = cv2.merge([bird_eye_gray, control.lidar_mask])
    #     # print"d"
    #     # final_image_16u = cv2.merge([final_image[:,:,0], final_image[:,:,1]]).astype(np.uint16)
    #     # print"e"
    #     # ros_image = bridge.cv2_to_imgmsg(final_image_16u, encoding="16UC2")  
    #     # print"f"
    #     # image_pub.publish(ros_image)
    #     bird_eye_gray_msg = self.bridge.cv2_to_imgmsg(bird_eye_gray, encoding="mono8")
    #     image_pub.publish(bird_eye_gray_msg)
    #     gray_msg = bridge.cv2_to_imgmsg(gray, encoding="mono8")
    #     gray_pub.publish(gray_msg)
    #     print"g"



class Lidar:
    def __init__(self):
        rospy.Subscriber("/scan", LaserScan, self.lidar_callback)
        print("lidar subscriber start")
        self.cal_mtx, self.cal_roi = cv2.getOptimalNewCameraMatrix(mtx, dist
                                    , (Width, Height), 1, (Width, Height))
        self.lidar_points = None
        self.lidar_mask = None     

    def lidar_callback(self, scan):
        self.lidar_points = scan.ranges
        # lidar_mask = self.lidar_to_mask(scan, self.cal_mtx)
        # print"C"
        # self.set_lidar_mask(lidar_mask)





    ################################################################################

    # Recognition: obstacle
    def is_obstacle_ahead(self, threshold=0.4, check_range=200, count_limit=5):
        countright = 0
        countleft = 0

        for deg in range(check_range+1):
            if 0.01 < self.lidar_points[deg+180] <= threshold:
                countright += 1
            if 0.01 < self.lidar_points[180-deg] <= threshold:
                countleft += 1

        if countright > count_limit and countleft < count_limit:
            return 1
        elif countright < count_limit and countleft > count_limit:
            return 2
        else:
            return 0

    # Recognition: tunnel
    def is_tunnel(self, threshold=0.4, check_range=40, count_limit=20):
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
        print"count1",count1
        print"count2",count2
        print"left lidar", self.lidar_points[0]
        print"right lidar", self.lidar_points[360]
        return (count1 > count_limit) and (count2 > count_limit)

    ################################################################################

    # Performing: when obastacle is right side
    def right_obstacle_driving(self):
        rtn = list()
        distance = 0
        count = 0
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
        return distance/5, i/2

    # Performing: when obastacle is left side
    def left_obstacle_driving(self):
        rtn = list()
        distance = 0
        count = 0
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
        return distance/5, i/2
        
    def tunnel_driving(self):
        left = list()
        right = list()
        for i in range(40):
            if not np.isinf(self.lidar_points[i]):
                left.append(self.lidar_points[i])
            if not np.isinf(self.lidar_points[360-i]):
                right.append(self.lidar_points[360-i])
        left_distance = sum(left)/len(left)
        right_distance = sum(right)/len(right)
        return left_distance, right_distance


    def lidar_to_mask(self, scan, cal_mtx):
        angles = np.linspace(scan.angle_min, scan.angle_max, len(scan.ranges))
        xs = np.array(scan.ranges) * np.cos(angles)
        ys = np.array(scan.ranges) * np.sin(angles)
        zs = np.zeros_like(xs)
        lidar_points = np.vstack((xs, ys, zs)).T

        lidar_cam = np.dot(R_lidar2cam, lidar_points.T) + T_lidar2cam
        lidar_cam = lidar_cam.T
        mask = lidar_cam[:, 2] > 0.1
        lidar_cam = lidar_cam[mask]

        pts_2d, _ = cv2.projectPoints(
            lidar_cam, 
            np.zeros((3,1)), 
            np.zeros((3,1)), 
            cal_mtx,
            None
        )
        pts_2d = pts_2d.reshape(-1, 2)

        lidar_mask = np.zeros((Height, Width), dtype=np.uint8)
        for pt in pts_2d:
            u, v = int(pt[0]), int(pt[1])
            if 0 <= u < Width and 0 <= v < Height:
                lidar_mask[v, u] = 1
        return lidar_mask

    def set_lidar_mask(self, lidar_mask):
        self.lidar_mask = lidar_mask
