#!/usr/bin/env python3

import rospy
import numpy as np
import cv2
import math
from queue import Queue
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
from std_msgs.msg import Float32

# automatic Canny edge detection 적용
def auto_canny(image, sigma=0.33):
    image_blur = cv2.GaussianBlur(image, (3, 3), 0)
    v = np.median(image_blur)
    lower = int(max(0, (1.0 - sigma) * v))
    upper = int(min(255, (1.0 + sigma) * v))
    edged = cv2.Canny(image, lower, upper)
    return edged

# 이미지 중 필요없는 부분 잘라내기
def image_crop(image, start_point=(0, 100), end_point=(640, 450)):
    mask = np.zeros(image.shape[:2], dtype=np.uint8)
    cv2.rectangle(mask, start_point, end_point, 255, thickness=cv2.FILLED)
    image_crop = cv2.bitwise_and(image, image, mask=mask)
    return image_crop

def draw_lines(img, lines, color=(0, 255, 0), thickness=3):
    if lines is not None:
        for line in lines:
            for x1, y1, x2, y2 in line:
                cv2.line(img, (x1, y1), (x2, y2), color, thickness)  # 검출된 선을 이미지 위에 그림

# x절편 구하는 함수
def get_x_coordinate(y_value, slope, y_intercept):
    return int((y_value - y_intercept) / slope)

def lane_detect(image):
    start_point = (100, 220)  # 자르는 시작위치 (왼쪽 위 점)
    end_point = (640, 450)  # 자르는 끝 위치 (오른쪽 아래 점)

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

    global slope_fit
    global y_intercept_fit
    global orientation
    global slope_avg

    y_value = 300

    if lines is not None:
        cv2.line(result_color, n, (cross_x, cross_y), (0, 0, 255), 1)
        cv2.line(result_color, n_2, (cross_x_2, cross_y_2), (0, 0, 255), 1)
        cv2.line(result_color, (right1, 0), (right1, 480), (0, 0, 255), 1)
        cv2.line(result_color, (right2, 0), (right2, 480), (0, 0, 255), 1)

        sum = 0
        i = 0

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
                i += 1
                sum += slope

        if i == 0:
            slope_avg = guide_slope
        elif i > 6:
            slope_avg = abs(np.arctan(sum / i))
        else:
            slope_avg = np.arctan(sum / i)
        print("slope_avg:", slope_avg)

    if len(filtered_points) > 0:
        points_arr = np.array(filtered_points)
        fitted_line = np.polyfit(points_arr[:, 0], points_arr[:, 1], deg=1)
        slope_fit, y_intercept_fit = fitted_line
        orientation = np.arctan(slope_fit)

        x1_fit = int(min(points_arr[:, 0]))
        y1_fit = int(slope_fit * x1_fit + y_intercept_fit)
        x2_fit = int(max(points_arr[:, 0]))
        y2_fit = int(slope_fit * x2_fit + y_intercept_fit)

        cv2.line(result_color, (x1_fit, y1_fit), (x2_fit, y2_fit), (255, 0, 0), 2)
        cv2.line(result_color, (0, y_value), (640, y_value), (0, 230, 200), 1)

    x_value = get_x_coordinate(y_value, slope_fit, y_intercept_fit)
    print("x_value:", x_value)

    steering_a = a * (guide_slope - slope_avg)
    if steering_a > 14:
        steering_a = 14
    if steering_a < -14:
        steering_a = -14
    steering_b = b * (guide_x - x_value)
    if steering_b > 14:
        steering_b = 14
    if steering_b < -14:
        steering_b = -14

    steering = steering_a + steering_b
    print('steering:', steering, 'steering_a:', steering_a, 'steering_b:', steering_b)

    if steering > 14:
        steering = 14
    if steering < -14:
        steering = -14

    if i > 5:
        if abs(steering) > 5:
            steering = 0

    return result_color, steering

class lane_masking:
    def __init__(self):
        rospy.init_node("lane_masking", anonymous=True)
        self.cv_bridge = CvBridge()

        rospy.Subscriber('/usb_cam/image_raw/compressed', CompressedImage, self._image_callback)
        self.steer_pub = rospy.Publisher('lane_detect', Float32, queue_size=1)
        self.lane_angle_pub = rospy.Publisher('lane_angle', Float32, queue_size=10)  # Publisher for lane angle

        self.lane_change_flag = 0

    def _image_callback(self, msg):
        image = self.cv_bridge.compressed_imgmsg_to_cv2(msg)
        if self.lane_change_flag == 1:
            image = image[:, ::-1, :]

        result, steering = lane_detect(image)
        cv2.imshow("result", result)
        cv2.waitKey(1)

        steer_msg = Float32()
        steer_msg.data = steering
        self.steer_pub.publish(steer_msg)

        # Publish the lane angle
        lane_angle_msg = Float32()
        lane_angle_msg.data = math.atan(steering)
        self.lane_angle_pub.publish(lane_angle_msg)

    def _lane_change_sub(self, msg):
        try:
            self.lane_change_flag = msg.data
        except:
            self.lane_change_flag = 0

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    Lane_masking = lane_masking()
    Lane_masking.run()
