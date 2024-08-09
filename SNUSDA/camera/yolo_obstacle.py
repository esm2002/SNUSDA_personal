#!/usr/bin/env python3

from ultralytics import YOLO
import torch
import cv2
from cv_bridge import CvBridge
import rospy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Int16
import copy
from time import strftime, localtime

class YoloDetection:

    def __init__(self, weights_path='/home/fmtc/catkin_ws/src/camera/obstacle_model.pt'): #FIXME
        self.load_model(weights_path)
        rospy.init_node('yolo_detection_2', anonymous=True) # yolo_detection라는 이름으로 노드 초기화
        self.bridge = CvBridge()
        # 신호등용 카메라에서 camera1/usb_cam1/image_raw/compressed topic을 subscribe
        self._bgr_sub = rospy.Subscriber('camera2/usb_cam2/image_raw/compressed', CompressedImage, self._image_callback) 
        # 신호등 color, bbox_info를 traffic_light_color_decoder_final.py에 publish
        self.obstacle_pub = rospy.Publisher('obstacle_detection', Int16,queue_size=2)

    def load_model(self, weights_path):
        self.model = YOLO(weights_path) # YOLO 모델 불러오기
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu') # GPU 사용 가능하면 GPU 사용
        self.model.to(self.device) # 모델을 GPU 메모리에 올리기
        

    def _image_callback(self, msg):
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
    def filter_bbox(self, cap_img, bbox_info, min_score_thresh=0.8, obstacle_label=0, size_standard=30000): 
        boxes = bbox_info.xyxy.type(torch.int) # bbox coordinates
        scores = bbox_info.conf # confidence score
        classes = bbox_info.cls # class label
        
        if(boxes.shape[0] == 0) : # if nothing detected
            print("!!!!nothing detected.") 
            self.obstacle_pub.publish(0)

            
        else : #something detected
                
            for i in range(boxes.shape[0]):  # boxes.shape[0] : number of boxes
                
                bbox_array = [pixel_val.item() for pixel_val in boxes[i]] # [xmin, ymin, xmax, ymax]
                [xmin, ymin, xmax, ymax] = bbox_array
                bbox_size = (xmax-xmin) * (ymax-ymin)
                
                if scores[i] > min_score_thresh and bbox_size>size_standard:
                    print("[YOLO node] Obstacle detected")
                    self.obstacle_pub.publish(xmin*10+1)
                
            

if __name__ == "__main__":   

    yolo_detection = YoloDetection()
    rospy.sleep(1)  # wait for the node to be initialized

    rate = rospy.Rate(10) # FIXME : check the rate   
    while not rospy.is_shutdown():
        
        cap_img, bbox_info = yolo_detection(plot_flag=True) # 이미지 및 bbox 정보 받아오기
        yolo_detection.filter_bbox(cap_img, bbox_info) # 해당 정보를 바탕으로 신호등 및 횡단보도 filtering & topic publish
    
        rate.sleep()
