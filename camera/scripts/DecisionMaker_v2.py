#!/usr/bin/env python3

'''
DecisionMaker_for_race.py

1. 카메라에서 받은 정보를 바탕으로 자율주행을 판단하는 코드
2. usb_cam 노드에서 publish한 topic(image_raw/compressed)을 subscribe
3. 이미지 처리(차선 검출)을 통해 조향값을 계산
4. 아두이노(serial_node)에 최종 명령(steer_value) publish
'''

import rospy
from std_msgs.msg import Int16
from sensor_msgs.msg import CompressedImage
import numpy as np
import cv2
from cv_bridge import CvBridge
from collections import deque

# Global Variables
frame = None
control = deque(maxlen=5)  # Maintain last 5 controls

# Lane detection functions
def auto_canny(image, sigma=0.33):
    image_blur = cv2.GaussianBlur(image, (3, 3), 0)
    v = np.median(image_blur)
    lower = int(max(0, (1.0 - sigma) * v))
    upper = int(min(255, (1.0 + sigma) * v))
    edged = cv2.Canny(image, lower, upper)
    return edged

def image_crop(image, start_point=(0, 100), end_point=(640, 450)):
    mask = np.zeros(image.shape[:2], dtype=np.uint8)
    cv2.rectangle(mask, start_point, end_point, 255, thickness=cv2.FILLED)
    image_crop = cv2.bitwise_and(image, image, mask=mask)
    return image_crop

def draw_lines(img, lines, color=(0, 255, 0), thickness=3):
    if lines is not None:
        for line in lines:
            for x1, y1, x2, y2 in line:
                cv2.line(img, (x1, y1), (x2, y2), color, thickness)

def get_x_coordinate(y_value, slope, y_intercept):
    return int((y_value - y_intercept) / slope)

def lane_detect(image):
    start_point = (100, 220)
    end_point = (640, 450)
    
    edge = auto_canny(image)
    edge_crop = image_crop(edge, start_point, end_point)
    lines = cv2.HoughLinesP(edge_crop, rho=1, theta=np.pi/180, threshold=20, minLineLength=70, maxLineGap=10)
    result = edge_crop.copy()
    result_color = cv2.cvtColor(result, cv2.COLOR_GRAY2BGR)
    
    filtered_points = []

    m = 0.3
    n = (620, 380)
    k = 700
    cross_x = n[0] - k
    cross_y = round(n[1] - k * m)
    
    m_2 = 2.5
    n_2 = (630, 400)
    cross_x_2 = n_2[0] - k
    cross_y_2 = round(n_2[1] - k * m_2)

    right1 = 330
    right2 = 640

    guide_x = 550
    guide_slope = 1.11

    a = 17
    b = 0.06
    
    y_value = 300
    slope_avg = guide_slope

    if lines is not None:
        cv2.line(result_color, n, (cross_x, cross_y), (0,0,255), 1)
        cv2.line(result_color, n_2, (cross_x_2, cross_y_2), (0, 0, 255), 1)
        cv2.line(result_color, (right1, 0), (right1, 480), (0,0,255), 1)
        cv2.line(result_color, (right2, 0), (right2, 480), (0,0,255), 1)
        
        sum_slope = 0
        count = 0

        for line in lines:
            x1, y1, x2, y2 = line[0]
            slope = (y2 - y1) / (x2 - x1)
            midpoint = ((x1 + x2) / 2, (y1 + y2) / 2)
            if ((midpoint[0] > right1) and (midpoint[0] < right2) and 
                (m * (midpoint[0] - n[0]) + (n[1] - midpoint[1]) > 0 and 
                (m_2 * (midpoint[0] - n_2[0]) + (n_2[1] - midpoint[1]) < 0) and 
                abs(slope) > 0.4)):
                filtered_points.append((x1, y1))
                filtered_points.append((x2, y2))
                cv2.line(result_color, (x1, y1), (x2, y2), (0, 255, 0), 2)
                count += 1
                sum_slope += slope

        if count > 0:
            slope_avg = np.arctan(sum_slope / count)
        x_value = get_x_coordinate(y_value, slope_avg, 0)
        
        steering_a = a * (guide_slope - slope_avg)
        steering_a = max(min(steering_a, 14), -14)
        steering_b = b * (guide_x - x_value)
        steering_b = max(min(steering_b, 14), -14)

        steering = steering_a + steering_b
        steering = max(min(steering, 14), -14)

        if count > 5:
            if abs(steering) > 5:
                steering = 0

        return result_color, int(steering)
    else:
        return result_color, 0

def image_callback(msg):
    global frame
    frame = bridge.compressed_imgmsg_to_cv2(msg, "bgr8") # Convert ROS image message to OpenCV format

def control_choose(prediction):
    control.append(prediction)
    if len(control) > 4:
        mean_val = (5*control[4] + 4*control[3] + 3*control[2] + 2*control[1] + control[0]) / 15
        if mean_val < 2:
            steer_val = round(-15 * mean_val + 30)
        else:
            steer_val = round(-12 * mean_val + 24)
        return steer_val
    else:
        return prediction

def Steer_publisher():
    rospy.init_node('decision_maker', anonymous=True)
    pub = rospy.Publisher('Steer_value', Int16, queue_size=10)
    rospy.Subscriber('/usb_cam/image_raw/compressed', CompressedImage, image_callback)
    
    global frame
    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        if frame is not None:
            result, steering = lane_detect(frame)
            current_control = control_choose(steering)
            pub.publish(current_control)
            rospy.loginfo(f'Steering value: {current_control}')
            cv2.imshow("Lane Detection", result)
            cv2.waitKey(1)
        rate.sleep()

if __name__ == "__main__":
    try:
        bridge = CvBridge()
        frame = None
        Steer_publisher()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()
