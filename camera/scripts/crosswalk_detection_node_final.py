#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32, Int32MultiArray
import time
"""
crosswalk_detection_node_final.py
1. yolo_final.py에서 publish한 topic(bbox/crosswalk)를 subscribe
2. bbox 좌표를 받아와서 거리 계산
3. 거리를 test_traffic_decision.py로 publish

* 현재는 사용하지 않음. 필요시 사용 가능
"""
class CrosswalkDetection:
    def __init__(self):
        self.crosswalk_distance = None
        self.bbox = []
        rospy.init_node('crosswalk_detection', anonymous=True)
        self._bbox_sub = rospy.Subscriber('/vision/bbox/crosswalk', Int32MultiArray, self._bbox_callback) # 횡단보도 bbox 좌표를 받아옴
        self._cw_distance_pub = rospy.Publisher('crosswalk_distance', Int32, queue_size=10) # 횡단보도와의 거리를 test_traffic_decision.py로 publish


    def _bbox_callback(self, bbox_coord_msg): # 횡단보도 bbox 좌표를 받아오는 callback 함수
        
        self.bbox = bbox_coord_msg.data
    

    def is_crosswalk_in_xbound(self, box_xyxy, bbox_center_bound=[0.1, 0.9], img_width=640): 
        # check whether the crosswalk is close to the car
        # by if the center of crosswalk bbox is in the box_position_threshold
        # box_position_threshold : [xmin, ymin, xmax, ymax]
        
        print(box_xyxy) # for debugging
        
        if(len(box_xyxy) == 0) : # 횡단보도가 감지되지 않았을 때
            return False
         
        else : # 횡단보도가 감지되었을 때 
            xmin = box_xyxy[0]
            xmax = box_xyxy[2]
            
            box_center_x = (xmin + xmax) / 2 # 횡단보도 bbox의 중심 x좌표
            
            if bbox_center_bound[0]*img_width < box_center_x < bbox_center_bound[1]*img_width: 
                return True
            else:
                return False


    def crosswalk_y_distance(self, box_xyxy, img_height=480): 
        # calculate the distance between the crosswalk and the car
        # by the center of crosswalk bbox
        [_, ymin, _, ymax] = box_xyxy
        box_distance_from_car = img_height - (ymin + ymax) / 2  # as y axis is flipped
        
        return round(box_distance_from_car)
    

    def publish_crosswalk_distance(self, distance): # 횡단보도와의 거리를 test_traffic_decision.py로 publish
        
        msg = Int32()
        msg.data = distance # 횡단보도와의 거리
        print(distance)
        self._cw_distance_pub.publish(msg)

        rospy.loginfo(msg)
    

if __name__ == '__main__':
    
    cw_detection = CrosswalkDetection()
    #rospy.sleep(1) # wait for the node to be initialized
    rate = rospy.Rate(10)
    
    while not rospy.is_shutdown():
        
        # by using bbox info from YOLO, detect crosswalk
        print("==========================================================")
        inf_start = time.time()
        bbox_xyxy = cw_detection.bbox # 횡단보도 bbox 좌표
        
        if cw_detection.is_crosswalk_in_xbound(bbox_xyxy): # 횡단보도가 충분한 거리에 있을 떄
            cw_detection.crosswalk_distance = cw_detection.crosswalk_y_distance(bbox_xyxy) # 횡단보도와의 거리 계산
        
        else : # 횡단보도가 충분한 거리에 없을 때
            cw_detection.crosswalk_distance = 1000000000 # 임의의 큰 값으로 설정  
            
        cw_detection.publish_crosswalk_distance(cw_detection.crosswalk_distance) # 횡단보도와의 거리를 test_traffic_decision.py로 publish

        # for debugging
        if cw_detection.crosswalk_distance < 1000000000:
            print("detect crosswalk")
            
        else :
            print("no crosswalk")

        print("CW judging time : ", (time.time() - inf_start) * 1000, "ms") # 판단시간 측정
        print("\n==========================================================\n")
        

        rate.sleep()