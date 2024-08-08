#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int16
from sensor_msgs.msg import CompressedImage
import pickle
import numpy as np
import cv2
from cv_bridge import CvBridge


def image_callback(msg):
    global frame
    frame = bridge.compressed_imgmsg_to_cv2(msg, "bgr8")

def save_images():
    file_path = "traf.pickle"
    with open(file_path, "wb") as file:
        pickle.dump(image, file)
    rospy.loginfo(f"Images saved to {file_path}")

      
def image_saver():
    # ROS 노드 초기화
    rospy.init_node('image_saver', anonymous=True)
    
    global image
    image = []

    # 노드 종료 시 이미지 저장하도록 설정
    rospy.on_shutdown(save_images)

    # Subscribe to the compressed image topic
    rospy.Subscriber('/usb_cam/image_raw/compressed', CompressedImage, image_callback)
    
    global frame
    frame = None
    
    while not rospy.is_shutdown():
        if frame is not None:
            image.append(frame)
        
        if cv2.waitKey(1) & 0xFF == ord('q'): 
            break

if __name__ == "__main__":
    try:
        bridge = CvBridge()
        frame = None
        image_saver()
    except rospy.ROSInterruptException:
        pass
