import rospy
import time
from xycar_msgs.msg import xycar_motor
from line import classify_line_region_and_lane, plot_lane_detection
i_error = 0.0
prev_error = 0.0
start_time = time.time()

def PID(center, kp, ki, kd):
    global start_time, prev_error, i_error
    end_time = time.time()
    dt = end_time - start_time
    start_time = end_time

    error = 320 - center
    derror = error - prev_error
    p_error = kp * error
    i_error += ki * error * dt
    d_error = kd * derror / dt if dt > 0 else 0

    output = p_error + i_error + d_error
    prev_error = error

    if output > 50:
        output = 50
    elif output < -50:
        output = -50

    return -output

def drive(pub, angle, speed):
    msg = xycar_motor()
    msg.angle = angle
    msg.speed = speed
    pub.publish(msg)

def process_and_control(pub):
    global final_image
    rate = rospy.Rate(10)
    stop_completed = False
    crosswalk_detected = False
    while not rospy.is_shutdown():
        if final_image is None:
            rate.sleep()
            continue

        result, lpos, rpos, left_lines, right_lines = classify_line_region_and_lane(final_image[:, :, 0])
        plot_lane_detection(final_image[:, :, 0], result, lpos, rpos, left_lines, right_lines, final_image[:, :, 1])
        center = int((lpos + rpos) / 2.0)   

        if result == "crosswalk" and not stop_completed:
            print("crosswalk")
            drive(pub, 0, 0)
            time.sleep(5)
            stop_completed = True
            crosswalk_detected = False
        elif result == "stop_line":
            print("stop line is detected")
        elif result == "start_line":
            print("start line is detected")

        angle = PID(center, 0.45, 0.0007, 0.25)
        drive(pub, angle, 5)
        rate.sleep()
