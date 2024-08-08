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

weights_path='/home/yunsu0915/Downloads/obstacle_model.pt'
model = YOLO(weights_path) # YOLO 모델 불러오기
device = torch.device('cuda' if torch.cuda.is_available() else 'cpu') # GPU 사용 가능하면 GPU 사용
model.to(device)

bridge = CvBridge()
bgr_img = cv2.imread('recent:///d35b3b2e3b534ae600c53531669ebb6f')

with torch.no_grad(): 

    # Convert image to tensor
    cap_img = copy.deepcopy(bgr_img) # ! IMPORTANT : prevent the image from being updated

    print("[Yolo Node] YOLO detected") 
    results = model(cap_img)[0] # 모델에 이미지를 입력하여 결과 받아오기
    bbox_info = results.boxes 



min_score_thresh=0.8
obstacle_label=0
size_standard=15000 

boxes = bbox_info.xyxy.type(torch.int) # bbox coordinates
scores = bbox_info.conf # confidence score
classes = bbox_info.cls # class label

if(boxes.shape[0] == 0) : # if nothing detected
    print("2") 

    
else : #something detected
        
    for i in range(boxes.shape[0]):  # boxes.shape[0] : number of boxes
        
        bbox_array = [pixel_val.item() for pixel_val in boxes[i]] # [xmin, ymin, xmax, ymax]
        [xmin, ymin, xmax, ymax] = bbox_array
        bbox_size = (xmax-xmin) * (ymax-ymin)
        print('siz: ',bbox_size)
        
        if scores[i] > min_score_thresh and classes[i] == obstacle_label and bbox_size>size_standard:
            print("1")
        else:
            print("0")


