#!/usr/bin/env python3

"""
traffic_light_color_decoder_final.py
1. 신호등 색 판단해 stop_flag 판단 및 bbox_size 계산 
2. yolo_final.py에서 publish한 topic(color, bbox_info)를 subscribe
3. test_traffic_light.py에 stop_flag, bbox_size msg publish

"""
import numpy as np
import cv2
from cv_bridge import CvBridge
import rospy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Int32, Int32MultiArray
import time
from time import strftime, localtime

from camera.msg import traffic_light_stop # traffic_light_stop.msg 파일 참고(custom msg) : stop_flag, bbox_size
from camera.msg import traffic_light_info # traffic_light_info.msg 파일 참고(custom msg) : color, bbox_info

class TrafficLightDecoder: 

    def __init__(self):

        rospy.init_node("traffic_node", anonymous=True) # traffic_node라는 이름으로 노드 초기화
        self.bridge = CvBridge() 
        
        self.cap_img = []
        self.bbox = []
        self.traffic_light_color = "nothing"

        # yolo_final.py에서 publish한 topic(color, bbox_info)를 subscribe
        self._bbox_sub = rospy.Subscriber('/vision/bbox/traffic_light', traffic_light_info, self._bbox_callback)

        # test_traffic_decision.py에 (stop_flag, bbox_size) publish
        self.traffic_flag_pub = rospy.Publisher("stop_signal", traffic_light_stop, queue_size=2) 
        

    def _bbox_callback(self, msg): # yolo_final.py에서 publish한 topic(color, bbox_info)를 subscribe하기 위한 callback 함수
        
        self.traffic_light_color = msg.color
        self.bbox = msg.bbox_info


    def decode_traffic_signal(self): # 신호등 색 판단하여 stop_flag, bbox_size return
        
        print(self.traffic_light_color)
        print(self.bbox)
        
        if len(self.bbox) == 0:  # bbox가 없으면 : 감지된 신호등이 없으면
            print("[Traffic Light Decoder] No traffic light detected") 
            return None
        
        else : # bbox가 있으면 : 감지된 신호등이 있으면

            print("[Traffic Light Decoder] traffic light detected")  
            
            if(self.traffic_light_color == 'red'): # 감지된 신호등이 red이면
                
                [xmin, ymin, xmax, ymax] = self.bbox
                bbox_size = (xmax-xmin) * (ymax-ymin) # bbox의 크기 계산
                print(bbox_size)
                
                stop_flag = 1 # 정지신호 전달
                
                return stop_flag, bbox_size
        
        stop_flag = 0 # 신호등이 감지되었으나 green인 경우, stop_flag = 0
        
        return stop_flag, 0
        
        
    def traffic_flag_publish(self): # stop_flag, bbox_size publish
        
        msg = traffic_light_stop()  #taffic_light_stop.msg 파일 참고 : stop_flag, bbox_size
        
        result = self.decode_traffic_signal() # stop_flag, bbox_size 

        if result is not None: # 감지된 신호등이 있으면
            msg.stop_flag, msg.bbox_size = result

        else: # 감지된 신호등이 없으면
            msg.stop_flag = 0
            msg.bbox_size = 0
        
        self.traffic_flag_pub.publish(msg) 


if __name__ == "__main__":
    traffic_decoder = TrafficLightDecoder()
    rospy.sleep(1)  # wait for node initialization

    rate = rospy.Rate(10)  # FIXME: check the rate

    while not rospy.is_shutdown():
        print("\n==========================================================\n")
        inf_start = time.time()

        # publish stop flag
        traffic_decoder.traffic_flag_publish()

        print("TL judging time : ", (time.time() - inf_start) * 1000, "ms") # 판단하는데 걸린 시간
        print("\n==========================================================\n")

        rate.sleep()
