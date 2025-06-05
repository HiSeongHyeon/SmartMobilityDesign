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

    if lines is None:
        return 0, Width

    # Reshape lines for convenience
    lines = [line[0] for line in lines]

    vertical_count = count_lines_by_slope(lines, 0.0, 0.1)
    diagonal_count = count_lines_by_slope(lines, 0.3, 2.0)
    horizontal_count = count_lines_by_slope(lines, 10.0, float('inf'))

    horizontal_line_detected = horizontal_count >= 3

    if horizontal_line_detected:
        if vertical_count >= 6:
            crosswalk_detected = True
            return "crosswalk"
        elif diagonal_count >= 3:
            return "stop_line"
        else:
            return "start_line"
    else:
        return "none"

def process_image(frame):
    result = classify_line_region(frame)
    return result

def start():
    global pub, info_pub, image, crosswalk_detected, stop_completed
    rospy.init_node('auto_drive')
    pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
    info_pub = rospy.Publisher('xycar_info', String, queue_size=1)
    rospy.Subscriber("/usb_cam/image_raw", Image, img_callback)
    print("---------- Xycar A2 시작 ----------")
    rospy.sleep(2)

    while not rospy.is_shutdown():
        if image.size != (640*480*3):
            continue

        result = process_image(image)

        if result == "crosswalk" and not stop_completed:
            info_pub.publish("횡단보도 정지! 5초 대기 중...")
            drive(0, 0)
            time.sleep(5) 
            stop_completed = True
            crosswalk_detected = False
        elif result == "stop_line":
            info_pub.publish("정지선 인식됨")
        elif result == "start_line":
            info_pub.publish("출발선 인식됨")

        center = Width // 2
        angle = PID(center, 0.45, 0.0007, 0.25)
        drive(angle, 5)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

if __name__ == '__main__':
    start()
