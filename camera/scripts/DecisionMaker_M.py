#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int16
from sensor_msgs.msg import CompressedImage
import pickle
import numpy as np
import cv2
from cv_bridge import CvBridge
import time
import tensorflow as tf
from collections import Counter
#model build

class ImageClassifierROSNode:
    def __init__(self):
        rospy.init_node('decision_maker', anonymous=True)
        rospy.Subscriber('/usb_cam/image_raw/compressed', CompressedImage, self.image_callback)
        self.image_pub = rospy.Publisher('lane_decision', Int16, queue_size=10)
        self.CvBridge = CvBridge()
        self.model = tf.keras.models.load_model("/home/isy/s1_ws/src/camera/scripts/final.h5") # 모델 위치 수정
        self.control = []
        self.last_control = 0

    def image_callback(self, data):
        try:
            frame = self.CvBridge.compressed_imgmsg_to_cv2(data, "bgr8")
            prediction = Int16()
            prediction.data = int(tf.argmax(self.model(frame.reshape(-1,480,640,3)),axis=1)) + 1
            current_control = self.control_choose(prediction.data)
            rospy.loginfo(f'Predicted class: {prediction}')
            if current_control != self.last_control:
                self.image_pub.publish(self.control_choose(prediction.data))
                self.last_control = current_control
            else: 
                self.last_control = current_control
        except Exception as e:
            rospy.logerr(f"Error processing the image: {e}")
    
    def control_choose(self, prediction):
        self.control.append(prediction)
        if len(self.control) > 10:
            self.control = self.control.pop(0)
        element_count = Counter(self.control)
        most_common_element, count = element_count.most_common(1)[0]
        return most_common_element

    

if __name__ == '__main__':
    try:
        classifier_node = ImageClassifierROSNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

