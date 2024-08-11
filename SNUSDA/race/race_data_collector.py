#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Int16
import cv2
import numpy as np
import os

class DataCollector:
    def __init__(self, file_base_name):
        rospy.init_node('data_collector', anonymous=True)

        self.image_dir = '/path/to/save/images'
        os.makedirs(self.image_dir, exist_ok=True)

        self.current_image = None
        self.steer_value = 0

        self.file_base_name = file_base_name

        rospy.Subscriber('/usb_cam/image_raw/compressed', CompressedImage, self.image_callback)
        rospy.Subscriber('steering_angle', Int16, self.steer_callback)

        self.counter = 0

    def image_callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        self.current_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    def steer_callback(self, msg):
        self.steer_value = msg.data

    def save_data(self):
        if self.current_image is not None and self.steer_value in [1, 2, 3]:  # 좌회전, 전진, 우회전 중인 경우에만 저장
            image_filename = os.path.join(self.image_dir, f'{self.file_base_name}_{self.counter:05d}.jpg')
            cv2.imwrite(image_filename, self.current_image)
            rospy.loginfo(f'Saved image: {image_filename}')

            steer_filename = os.path.join(self.image_dir, f'{self.file_base_name}_{self.counter:05d}.txt')
            with open(steer_filename, 'w') as f:
                f.write(f'{self.steer_value}\n')
            rospy.loginfo(f'Saved steer value: {steer_filename} with value {self.steer_value}')

            self.counter += 1

    def run(self):
        rate = rospy.Rate(10)  # 초당 10회의 주기로 실행 (카메라 프레임 레이트와 일치)
        while not rospy.is_shutdown():
            self.save_data()
            rate.sleep()

if __name__ == '__main__':
    try:
        file_base_name = input("Enter the base name for the files: ")
        collector = DataCollector(file_base_name)
        collector.run()
    except rospy.ROSInterruptException:
        pass
