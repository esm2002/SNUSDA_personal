#!/usr/bin/env python3

import rospy

import numpy as np
import cv2
import math
from queue import Queue

from sensor_msgs.msg import CompressedImage
from lidar.msg import obstacle_detection
from cv_bridge import CvBridge
from std_msgs.msg import Float32

# automatic Canny edge detection 적용
def auto_canny(image, sigma=0.33):
    
    image_blur = cv2.GaussianBlur(image, (3, 3), 0)
    # compute the median of the single channel pixel intensities
    v = np.median(image_blur)
    # apply automatic Canny edge detection using the computed median
    lower = int(max(0, (1.0 - sigma) * v))
    upper = int(min(255, (1.0 + sigma) * v))
    edged = cv2.Canny(image, lower, upper)
    
    return edged

# 이미지 중 필요없는 부분 잘라내기
def image_crop(image, start_point = (0, 100), end_point=(640,450)):

    mask = np.zeros(image.shape[:2], dtype=np.uint8)
    cv2.rectangle(mask, start_point, end_point, 255, thickness= cv2.FILLED)
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
  
def lane_detect(image) : 
    hsv= cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    
    lower_green=np.array([30,80,80])
    upper_green=np.array([70,255,255])
    mask_green=cv2.inRange(hsv, lower_green, upper_green)
    #print('green: ' , mask_green[-1,-1])

    lower_white = np.array([0, 0, 180])
    upper_white = np.array([180, 25, 255])
    mask_white = cv2.inRange(hsv, lower_white, upper_white)
    #print('white: ' , mask_white[-1,-1])
    
    if mask_green[-1,-1]==255:
    	steerting=15 # green, 왼쪽
    	
    elif mask_white[-1,-1]==255:
    	steering=0 # white, 직진
    else:
    	steering=-15 # gray, 오른쪽
          
  
  
class lane_masking : 
  
  def __init__ (self) :
    
    rospy.init_node("lane_masking", anonymous=True) # 노드 초기화
    self.cv_bridge = CvBridge()
  
    rospy.Subscriber("/camera1/usb_cam1/image_raw/compressed", CompressedImage, self._image_callback) # usbcam으로부터 img 데이터 subscribe

    self.steer_pub = rospy.Publisher('lane_detect', Float32, queue_size=1) # steering 데이터 publish
    self.lane_change_flag = 0
  
  def _image_callback(self, msg): # 불러온 img를 cv2 image 로 변환
    image = self.cv_bridge.compressed_imgmsg_to_cv2(msg)
    
    if self.lane_change_flag == 1: # 장애물 감지 주행중
       image = image[:,::-1,:]
       
    steering = lane_detect(image)
    
    steer_msg = Float32() 
    steer_msg.data = steering # publish할 메세지에 steering 값 부여

    self.steer_pub.publish(steer_msg) # steer_msg publish

  def _lane_change_sub(self, msg):
     try:
        self.lane_change_flag = msg.data # 장애물 감지 시
     except:
        self.lane_change_flag = 0   # 장애물 미 감지 시
        
  def run(self):
    rospy.spin()

if __name__ == "__main__":
  
    Lane_masking = lane_masking()
    Lane_masking.run() # run 함수 호출
