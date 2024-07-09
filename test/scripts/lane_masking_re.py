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
    
    start_point = (100, 220) # 자르는 시작위치 (왼쪽 위 점)
    end_point = (640,450)  # 자르는 끝 위치 (오른쪽 아래 점)
    
    edge = auto_canny(image)

    edge_crop = image_crop(edge, start_point, end_point)
    #cv2_imshow(edge_crop)

    # Hough 변환을 사용하여 직선 정보 검출
    lines = cv2.HoughLinesP(edge_crop, rho=1, theta=np.pi/180, threshold=20, minLineLength=70, maxLineGap=10)
    result = edge_crop.copy()
    result_color = cv2.cvtColor(result, cv2.COLOR_GRAY2BGR)
    
    #추출한 점 담는 새로운 리스트
    filtered_points = []

    ############### 관심 영역 지정을 위한 상수 ###############

    # 사선 위에 존재
    m = 0.3 # 사선 기울기
    n = (620, 380) # 사선이 지나는 점
    k = 700 
    cross_x = n[0]-k
    cross_y = round(n[1]-k*m)
    
    m_2 = 2.5 # 사선 기울기
    n_2 = (630, 400) # 사선이 지나는 점
    cross_x_2 = n_2[0]-k
    cross_y_2 = round(n_2[1]-k*m_2)

    # 수직선 기준 오른쪽 존재
    right1 = 330
    right2 = 640

    #########################################################

    ############### 조향값 조절을 위한 상수 ###################

    guide_x = 550 # 디폴트 x 값
    guide_slope = 1.11 # 디폴트 기울기

    a=17 # 기울기에 따른 조향값 조절 시 가중치
    b=0.06 # 절편에 따른 조향값 조절 시 가중치
    
    global slope_fit # fitting된 직선의 기울기
    global y_intercept_fit  # fitting된 직선의 y 절편 
    global orientation # 차량의 x방향으로의 위치
    global slope_avg # 검출된 차선 기울기 평균

    ###########################################################

    y_value = 300

    if lines is not None:  # lines가 None인 경우 검출된 직선이 없음
        # 디폴트로 그려질 가이드 선
        cv2.line(result_color, n, (cross_x, cross_y), (0,0,255), 1)  #사선
        cv2.line(result_color, n_2, (cross_x_2, cross_y_2), (0, 0, 255), 1) # 사선
        cv2.line(result_color, (right1, 0), (right1, 480), (0,0, 255), 1)  #왼쪽 수직선
        cv2.line(result_color, (right2, 0), (right2, 480), (0,0, 255), 1)   #오른쪽 수직선
        
        sum = 0
        i = 0
        
        # 필터링 된 차선을 이미지 위에 표시
        for line in lines:
            x1, y1, x2, y2 = line[0]
            slope = (y2-y1)/(x2-x1)
            
            #print('slope', slope)
            midpoint = ((x1+x2)/2, (y1+y2)/2) # 검출된 선의 중점 구하기
            #cv2.line(result_color, (x1, y1), (x2, y2), (0, 255, 0), 1)
            if ((midpoint[0]>right1) and (midpoint[0]<right2) and (m*(midpoint[0]-n[0])+(n[1]-midpoint[1]) > 0 and \
                (m_2*(midpoint[0]-n_2[0])+(n_2[1]-midpoint[1]) < 0 )and \
                  abs(slope) > 0.4)):  # 검출된 선의 중점이 관심영역 내부에 있으면
              filtered_points.append((x1, y1)) # array에 점 추가
              filtered_points.append((x2, y2))
              cv2.line(result_color, (x1, y1), (x2, y2), (0, 255, 0), 2) # 검출된 직선 이미지 위에 표시
              
              # 평균 기울기 구하기 위해 각 선의 기울기를 모두 더해줌
              i+=1
              sum += slope 

        if(i == 0) :  # 검출된 선이 없는 경우
          slope_avg = guide_slope
          #x_value = 600
        
        elif(i>6) : # 너무 많은 선이 검출된 경우(횡단보도)
           slope_avg = abs(np.arctan(sum/i)) #slope_avg를 절대값으로(가끔 음수로 추정되는 경우가 있었음)

        else :      
          slope_avg = np.arctan(sum/i)
          #x_value = sum2/i  
        print("slope_avg : ", slope_avg)


    
    if len(filtered_points) > 0: # 검출된 직선이  있는 경우
        points_arr = np.array(filtered_points)
        fitted_line = np.polyfit(points_arr[:, 0], points_arr[:, 1], deg=1) # 검출한 직선의 점들을 선형보간으로 fitting

        # fitting된 직선의 기울기와 y절편을 가져옵니다.
        slope_fit, y_intercept_fit = fitted_line
        orientation = np.arctan(slope_fit)

        # fitting된 직선의 두 점을 계산합니다.
        x1_fit = int(min(points_arr[:, 0]))
        y1_fit = int(slope_fit * x1_fit + y_intercept_fit)
        x2_fit = int(max(points_arr[:, 0]))
        y2_fit = int(slope_fit * x2_fit + y_intercept_fit)

        # fitting된 직선을 그립니다.
        cv2.line(result_color, (x1_fit, y1_fit), (x2_fit, y2_fit), (255, 0, 0), 2)
        cv2.line(result_color,(0,y_value),(640,y_value),(0,230,200),1)

    #print("orientation : ", orientation)
    #print("slope : ", slope_fit)
    x_value = get_x_coordinate(y_value, slope_fit, y_intercept_fit) # x 절편 구하기
    
    print("x_value : ", x_value)
    #print("\n")
    
    # 기울기와 절편에 따른 조향값 부여
    steering_a = a*(guide_slope - slope_avg)
    if steering_a > 14 : 
      steering_a = 14
    if steering_a < -14 :
      steering_a = -14
    steering_b = b* (guide_x- x_value)
    if steering_b > 14 :
      steering_b = 14
    if steering_b < -14 :
      steering_b = -14

    steering = steering_a + steering_b
    print('steering :', steering, 'steering_a :', steering_a, 'steering_b :', steering_b)
    
    # 출력이 최대 조향각을 넘을 경우
    if(steering > 14) :
      steering = 14
      
    if(steering < -14) :
      steering = -14

    # 너무 많은 차선이 검출되는 경우(횡단보도)
    if(i>5) :
       if abs(steering)>5:
          steering = 0 # 너무 크게 조향하지 않도록 (횡단보도가 있는 상황에서 직진을 할 수 있도록 함)

    # if (i==0) :
    #    steering = -8

    return result_color, steering
          
  
  
class lane_masking : 
  
  def __init__ (self) :
    
    rospy.init_node("lane_masking", anonymous=True) # 노드 초기화
    self.cv_bridge = CvBridge()
  
    rospy.Subscriber("/camera2/usb_cam2/image_raw/compressed", CompressedImage, self._image_callback) # usbcam으로부터 img 데이터 subscribe

    self.steer_pub = rospy.Publisher('lane_detect', Float32, queue_size=1) # steering 데이터 publish
    self.lane_change_flag = 0
  
  def _image_callback(self, msg): # 불러온 img를 cv2 image 로 변환
    
    # ! self.rgb_img : shape=(480, 640, 3), dtype=uint8, ndarray, BGR encoding
    image = self.cv_bridge.compressed_imgmsg_to_cv2(msg)
    # img = cv2.imread(image)
    # cv2.imshow("img", image)
    if self.lane_change_flag == 1: # 장애물 감지 주행중
       image = image[:,::-1,:]
       
    result, steering = lane_detect(image)
    cv2.imshow("result", result)
    cv2.waitKey(1)
    
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