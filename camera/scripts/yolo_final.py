#!/usr/bin/env python3

'''
yolo_final.py

1. YOLO 모델을 불러와 신호등과 횡단보도를 탐지하는 코드
2. traffic_light_color_decoder_final.py로 color, bbox_info msg 전달

'''

from ultralytics import YOLO
import torch
import cv2
from cv_bridge import CvBridge
import rospy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Int32MultiArray
import copy
from time import strftime, localtime

from camera.msg import traffic_light_info # traffic_light_info.msg 파일 참고(custom msg) : color, bbox_info

# 새로 라벨링한 class 정보 
    # cross-walk = 0
    # traffic-light-green = 1
    # traffic-light-red = 2
    # traffic-light-yellow = 3

class YoloDetection:

    def __init__(self, weights_path='/home/fmtc/catkin_ws/src/camera/scripts/best_0808.pt'): 
        
        #FIXME : change the path(모델 위치)
        self.load_model(weights_path)

        rospy.init_node('yolo_detection', anonymous=True) # yolo_detection라는 이름으로 노드 초기화
        
        self.bridge = CvBridge()

        # 신호등용 카메라에서 camera1/usb_cam1/image_raw/compressed topic을 subscribe
        self._bgr_sub = rospy.Subscriber('camera1/usb_cam1/image_raw/compressed', CompressedImage, self._image_callback) 

        # 신호등 color, bbox_info를 traffic_light_color_decoder_final.py에 publish
        self._tl_bbox_pub = rospy.Publisher('/vision/bbox/traffic_light', traffic_light_info, queue_size=2)

        # 횡단보도 bbox_info를 crosswalk_detection_node_final.py에 publish
        self._cw_bbox_pub = rospy.Publisher('/vision/bbox/crosswalk', Int32MultiArray, queue_size=2)


    def load_model(self, weights_path):
        # fine-tuned with cross-walk(class 0), traffic-light-green(class 1), traffic-light-red(class 2), traffic-light-yellow(class 3) dataset

        self.model = YOLO(weights_path) # YOLO 모델 불러오기
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu') # GPU 사용 가능하면 GPU 사용
        self.model.to(self.device) # 모델을 GPU 메모리에 올리기


    def _image_callback(self, msg): # compressed image를 cv2 image로 변환하는 callback 함수

        # ! self.bgr_img : shape=(480, 640, 3), dtype=uint8, ndarray, BGR encoding
        self.bgr_img = self.bridge.compressed_imgmsg_to_cv2(msg)

    def __call__(self,plot_flag=True): # YOLO 모델을 통해 신호등 및 횡단보도를 탐지하는 함수
        """
        Params
            plot_flag : if True, then plot the detected image
        
        Return
            cap_img : BGR encoded image input
            bbox_info : class 'ultralytics.engine.results.Boxes'
        """
        with torch.no_grad(): 

            # Convert image to tensor
            cap_img = copy.deepcopy(self.bgr_img) # ! IMPORTANT : prevent the image from being updated

            # Run object detection
            # results : class 'ultralytics.engine.results.Results'
            print("[Yolo Node] YOLO detected") 
            results = self.model(cap_img)[0] # 모델에 이미지를 입력하여 결과 받아오기
            bbox_info = results.boxes 

            # Code for debugging
            plots = results.plot()
            if plot_flag is True:
                # debug whether yolo detects the traffic light well
                cv2.imshow("[Yolo Detection] YOLO detected", plots)
                cv2.waitKey(500)

        return cap_img, bbox_info
    

    # 사진에서 신호등 및 횡단보도 bbox 정보를 추출하는 함수
    def filter_bbox(self, cap_img, bbox_info, min_score_thresh_tl=0.3, min_score_thresh_cw=0.5, 
                                                traffic_light_green_label=1, traffic_light_red_label = 2, traffic_light_yellow_label=3, crosswalk_label=0): 
        
        # FIXME 신호등 min_confidece score = 0.3, 횡단보도 min_confidence score = 0.5
        
        # filter bbox with score threshold, class label that we are interested in (traffic light, crosswalk)
        boxes = bbox_info.xyxy.type(torch.int) # bbox coordinates
        scores = bbox_info.conf # confidence score
        classes = bbox_info.cls # class label
        
        topic_to_publish = [] # 감지된 물체 정보 저장할 list
        bbox = [] # 감지된 물체의 bbox 정보 저장할 list
        
        if(boxes.shape[0] == 0) : # if nothing detected
            print("!!!!nothing detected.") 
            topic_to_publish.append('nothing') 
            bbox.append([]) 

            
        else : # 신호등 또는 횡단보도가 감지되었을 때
                
            for i in range(boxes.shape[0]):  # boxes.shape[0] : number of boxes
                
                # for bbox of traffic light and crosswalk, publish the xyxy bbox coordinates list
                  # ! each item dtype is float32 not torch.float
                
                bbox_array = [pixel_val.item() for pixel_val in boxes[i]] # [xmin, ymin, xmax, ymax]
                
                if scores[i] > min_score_thresh_tl and classes[i] == traffic_light_green_label: # 특정 confidence score 이상이고, class label이 traffic_light_green(1)인 경우
                    print("[YOLO node] Green traffic light detected")
                    topic_to_publish.append('TL_green') 
                    bbox.append(bbox_array)    
                    
                if scores[i] > min_score_thresh_tl and classes[i] == traffic_light_red_label: # 특정 confidence score 이상이고, class label이 traffic_light_red(2)인 경우
                    #print(classes[i])
                    print("[YOLO node] Red traffic light detected")
                    topic_to_publish.append('TL_red')
                    bbox.append(bbox_array)
                    
                    
                if scores[i] > min_score_thresh_tl and classes[i] == traffic_light_yellow_label: # 특정 confidence score 이상이고, class label이 traffic_light_yellow(3)인 경우
                    print("[YOLO node] Yellow traffic light detected")
                    topic_to_publish.append('TL_yellow')
                    bbox.append(bbox_array)
                    
                    
                if scores[i] > min_score_thresh_cw and classes[i] == crosswalk_label : # 특정 confidence score 이상이고, class label이 crosswalk(0)인 경우
                    print("[YOLO node] crosswalk detected")
                    topic_to_publish.append('CW')
                    bbox.append(bbox_array)
                    

        self.publish_bbox(topic_to_publish, bbox) # publish_bbox 함수 호출
                
        print("\n")




    def publish_bbox(self, topic_to_publish, bbox_for_publish): # color, bbox_info publish 함수

        print(topic_to_publish) # for debugging
        print(bbox_for_publish) # for debugging
        
        tl = 0 # 감지된 신호등 개수
        cw = 0 # 감지된 횡단보도 개수
        
        for i in range(len(topic_to_publish)) : # 감지된 물체 개수만큼 반복
            
            if(topic_to_publish[i] == 'nothing') : # 감지된 물체가 없으면
                
                self._cw_bbox_pub.publish(Int32MultiArray(data=[])) # 횡단보도 bbox publish = [] (빈 list)
                self._tl_bbox_pub.publish(traffic_light_info(color = "nothing", bbox_info = [])) # 신호등 color, bbox_info publish = "nothing", []
                
            if(topic_to_publish[i] == 'TL_green' or topic_to_publish[i] == 'TL_red' or topic_to_publish[i] == 'TL_yellow') : # 신호등이 감지되었으면
                
                traffic_light_info_msg = traffic_light_info() # traffic_light_info.msg 파일 참고 : color, bbox_info
            
                traffic_light_info_msg.bbox_info = bbox_for_publish[i] # 신호등 bbox 정보 저장
                traffic_light_info_msg.color = topic_to_publish[i] # 신호등 color 정보 저장
                
                tl+=1 # 신호등 개수 +1
                
                self._tl_bbox_pub.publish(traffic_light_info_msg) # 신호등 color, bbox_info publish
                
                if(cw == 0) : # 횡단보도가 감지되지 않았으면
                    self._cw_bbox_pub.publish(Int32MultiArray(data=[])) # 횡단보도 bbox publish = [] (빈 list)
                
            if(topic_to_publish[i] == 'CW') : # 횡단보도가 감지되었으면
        
                cw_bbox_msg = Int32MultiArray() 
                cw_bbox_msg.data = bbox_for_publish[i] # 횡단보도 bbox 정보 저장
                self._cw_bbox_pub.publish(cw_bbox_msg) # 횡단보도 bbox publish
                
                cw+=1 # 횡단보도 개수 +1
                
                if(tl == 0) : # 신호등이 감지되지 않았으면
                    self._tl_bbox_pub.publish(traffic_light_info(color = "nothing", bbox_info = [])) # 신호등 color, bbox_info publish = "nothing", []
        

if __name__ == "__main__":   

    yolo_detection = YoloDetection()
    rospy.sleep(1)  # wait for the node to be initialized

    rate = rospy.Rate(10) # FIXME : check the rate   
    while not rospy.is_shutdown():
        
        cap_img, bbox_info = yolo_detection(plot_flag=True) # 이미지 및 bbox 정보 받아오기
        yolo_detection.filter_bbox(cap_img, bbox_info) # 해당 정보를 바탕으로 신호등 및 횡단보도 filtering & topic publish
    
        rate.sleep()
