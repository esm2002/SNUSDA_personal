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

def pot_callback(data):
    global angle
    angle = data.data

def keyboard_publisher():
    # ROS 노드 초기화
    rospy.init_node('keyboard_publisher', anonymous=True)

    # 30Hz로 루프를 실행하면서 키보드 입력을 받아서 게시
    rate = rospy.Rate(10)
    msg = Int16()
    steering = []
    image = []
    
    print("1 : left / 2 : front / 3 : right / other : stop")

    # Subscribe to the compressed image topic
    rospy.Subscriber('/usb_cam/image_raw/compressed', CompressedImage, image_callback)
    rospy.Subscriber('Pot_value', Int16, pot_callback)

    while not rospy.is_shutdown():
        # msg.data = int(input("Enter a keyboard message: "))
        if frame is not None:
            image.append(frame)
        
        if angle is not None:
            steering.append(angle)
        
        # 4 눌러서 종료하기
        if msg.data == 4:
            combined_data = [steering, image]
            file_path = "data.pickle"
            with open(file_path, "wb") as file:
                pickle.dump(combined_data, file)
            break
        
        if cv2.waitKey(1) & 0xFF == ord('q'): 
            break

        # 루프 지연
        rate.sleep()

if __name__ == "__main__":
    try:
        bridge = CvBridge()
        frame = None
        angle = None
        # keyboard_publisher()
    except rospy.ROSInterruptException:
        pass
