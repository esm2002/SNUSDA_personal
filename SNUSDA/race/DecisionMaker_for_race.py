#!/usr/bin/env python3

'''
DecisionMaker_for_race.py

1. 카메라에서 받은 정보를 바탕으로 자율주행을 판단하는 코드
2. usb_cam 노드에서 publish한 topic(image_raw/compressed)을 subscribe
3. 딥러닝을 통해 image로부터 조향값(가변저항)을 계산
5. 아두이노(serial_node)에 최종 명령(steer_value) publish

'''

import rospy
from std_msgs.msg import Int16
from sensor_msgs.msg import CompressedImage
import numpy as np
import cv2
from cv_bridge import CvBridge
import time
import tensorflow as tf
from collections import Counter


control = []

def image_callback(msg):
    global frame
    frame = bridge.compressed_imgmsg_to_cv2(msg, "bgr8") # Change Imgage_msg to image_array(cv2)

def control_choose(prediction): # weighted average for recent 5 frames
    control.append(prediction)
    if len(control) > 4:
        mean_val = (5*control[4] + 4*control[3] + 3*control[2] + 2*control[1] + control[0])/15
        if mean_val < 2: # linear interpoloation : left_max = 22, middle = 0, right_max = -15
            steer_val = round(-22*mean_val + 44)
        else:
            steer_val = round(-15*mean_val + 30)
        
        control.pop(0)

        return steer_val
    else:
        return prediction # return just prediction if frames < 5
      
def Steer_publisher():
    
    rospy.init_node('decision_maker', anonymous=True) # init node

   
    pub = rospy.Publisher('Steer_value', Int16, queue_size=10) # define topic

    
    rospy.Subscriber('/usb_cam/image_raw/compressed', CompressedImage, image_callback) # get image from usb_cam node
    model = tf.keras.models.load_model("/home/fmtc/catkin_ws/src/race/scripts/finemodel_v2.h5") # Our CNN model load (trained with our data)
    rate = rospy.Rate(32)

    while not rospy.is_shutdown():
        prediction = Int16()
        prediction.data = int(tf.argmax(model(frame.reshape(-1,480,640,3)),axis=1)) + 1 # decide direction from image by CNN model 
        # model outputs are 0 for left, 1 for middle, 2 for right. +1 to make them (1,2,3)
        current_control = control_choose(prediction.data) # post processing, calculate steer_value
        pub.publish(current_control) # publish steer_value to arduino
        rospy.loginfo(f'Predicted class: {current_control}')

        rate.sleep()
        

if __name__ == "__main__":
    try:
        bridge = CvBridge()
        frame = None
        Steer_publisher()

    except rospy.ROSInterruptException:
        pass
