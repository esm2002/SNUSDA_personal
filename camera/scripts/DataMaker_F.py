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

      
def keyboard_publisher():
    # ROS 노드 초기화
    rospy.init_node('keyboard_publisher', anonymous=True)

    # 'keyboard_input' 토픽에 메시지를 게시하는 Publisher 생성
    pub = rospy.Publisher('keyboard_input', Int16, queue_size=10)

    # 30Hz로 루프를 실행하면서 키보드 입력을 받아서 게시
    #rate = rospy.Rate(30)
    msg = Int16()
    signal = Int16()
    control = []
    image = []
    
    print("1 : left / 2 : front / 3 : right / other : stop")

    # Subscribe to the compressed image topic
    rospy.Subscriber('/usb_cam/image_raw/compressed', CompressedImage, image_callback)
    

    while not rospy.is_shutdown():
        
        msg = int(input("Enter a keyboard message: "))

        if msg != signal.data:
            signal.data = msg
            pub.publish(signal)

    
        if frame is not None:
            
            image.append(frame)
         
        
        if msg is not None:
            
            control.append(msg)
        
        # 4 눌러서 종료하기
        if msg == 4:
            combined_data = [control, image]
            file_path = "newdata157.pickle"
            with open(file_path, "wb") as file:
                pickle.dump(combined_data, file)
            break
        
        if cv2.waitKey(1) & 0xFF == ord('q'): 
            break

        # 루프 지연
        #rate.sleep()

if __name__ == "__main__":
    try:
        bridge = CvBridge()
        frame = None
        keyboard_publisher()
    except rospy.ROSInterruptException:
        pass

