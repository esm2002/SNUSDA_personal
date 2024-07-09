#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int16
from sensor_msgs.msg import CompressedImage
import numpy as np
import cv2
from cv_bridge import CvBridge
import time
import tensorflow as tf
from collections import Counter

#model build

# model = tf.keras.models.load_model("/home/bgbong/future_car_ws/src/test/scripts/UNUS_Cam/best_weights101.h5")

control = []

def image_callback(msg):
    global frame
    frame = bridge.compressed_imgmsg_to_cv2(msg, "bgr8")

def control_choose(prediction):
    control.append(prediction)
    if len(control) > 7:
        mean_val = sum(control)/len(control)

        if mean_val < 2:
            steer_val = round(-18*mean_val + 36)
        else:
            steer_val = round(-16*mean_val + 32)
        # print(mean_val)
        # element_count = Counter(control)
        # print(element_count)
        #most_common_element = Int16()
        # most_common_element, count = element_count.most_common(1)[0]
        # print('most_commom_element : ', most_common_element)
        control.pop(0)

        return steer_val
    else:
        return prediction
      
def Steer_publisher():
    # ROS 노드 초기화
    rospy.init_node('decision_maker', anonymous=True)

    # 'keyboard_input' 토픽에 메시지를 게시하는 Publisher 생성
    pub = rospy.Publisher('Steer_value', Int16, queue_size=10)

    # Subscribe to the compressed image topic
    rospy.Subscriber('/usb_cam/image_raw/compressed', CompressedImage, image_callback)
    model = tf.keras.models.load_model("/home/movingwa1k/Desktop/best_f.h5")
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        prediction = Int16()
        prediction.data = int(tf.argmax(model(frame.reshape(-1,480,640,3)),axis=1)) + 1
        current_control = control_choose(prediction.data)
        pub.publish(current_control)
        rospy.loginfo(f'Predicted class: {current_control}')

        rate.sleep()
        

        # 루프 지연
        #rate.sleep()

if __name__ == "__main__":
    try:
        bridge = CvBridge()
        frame = None
        Steer_publisher()

    except rospy.ROSInterruptException:
        pass
