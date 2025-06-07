import numpy as np
import cv2
from config import Width, Height, R_lidar2cam, T_lidar2cam, mtx, dist, src_pts, dst_pts



def undistort_and_birdseye(raw_image, cal_mtx, cal_roi):
    undistorted = cv2.undistort(raw_image, mtx, dist, None, cal_mtx)
    x, y, w, h = cal_roi
    undistorted = undistorted[y:y + h, x:x + w]
    undistorted = cv2.resize(undistorted, (Width, Height))
    gray = cv2.cvtColor(undistorted, cv2.COLOR_BGR2GRAY)
    M_perspective = cv2.getPerspectiveTransform(src_pts, dst_pts)
    bird_eye_gray = cv2.warpPerspective(gray, M_perspective, (Width, Height))
    return bird_eye_gray


def lidar_to_mask(scan, cal_mtx):
    angles = np.linspace(scan.angle_min, scan.angle_max, len(scan.ranges))
    xs = np.array(scan.ranges) * np.cos(angles)
    ys = np.array(scan.ranges) * np.sin(angles)
    zs = np.zeros_like(xs)
    lidar_points = np.vstack((xs, ys, zs)).T

    lidar_cam = (R_lidar2cam @ lidar_points.T) + T_lidar2cam
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
