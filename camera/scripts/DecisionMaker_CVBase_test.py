#!/usr/bin/env python3

'''
DecisionMaker_for_race.py

1. 카메라에서 받은 정보를 바탕으로 자율주행을 판단하는 코드
2. usb_cam 노드에서 publish한 topic(image_raw/compressed)을 subscribe
3. 차선 각도 데이터를 통해 조향값을 계산하여 퍼블리시
4. 아두이노(serial_node)에 최종 명령(steer_value) publish
'''

import rospy
from std_msgs.msg import Float32, Int16
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import threading

# 글로벌 변수
frame = None
steering_value = None
control = []

class DecisionMaker:
    def __init__(self):
        rospy.init_node('decision_maker', anonymous=True)

        # 차선 각도 결과를 구독합니다.
        rospy.Subscriber('lane_angle_msg', Float32, self._lane_angle_callback)

        # 최종 조향값을 퍼블리시할 퍼블리셔를 정의합니다.
        self.steer_pub = rospy.Publisher('steering_value', Int16, queue_size = 10)

        # 최근 조향값을 저장할 버퍼를 초기화합니다.
        self.steer_buffer = []
        self.max_buffer_size = 5

        # 조향값을 계산하기 위한 상수
        self.max_steering = 22
        self.min_steering = -15

    def _lane_angle_callback(self, msg):
        global steering_value
        angle = msg.data

        # 각도에 기반하여 조향값을 계산합니다.
        steering = self.calculate_steering(angle)

        # 최신 조향값을 버퍼에 저장합니다.
        self.steer_buffer.append(steering)

        # 버퍼 크기를 유지합니다.
        if len(self.steer_buffer) > self.max_buffer_size:
            self.steer_buffer.pop(0)

        # 버퍼의 평균을 사용하여 부드러운 조향값을 계산합니다.
        if len(self.steer_buffer) > 0:
            smooth_steering = sum(self.steer_buffer) / len(self.steer_buffer)
        else:
            smooth_steering = 0

        # 부드러운 조향값을 퍼블리시합니다.
        steering_value = int(smooth_steering)  # Store the calculated steering value

        rospy.loginfo(f'차선 각도: {angle}, 조향값: {steering}, 부드러운 조향값: {smooth_steering}')

    def calculate_steering(self, angle):
        # 각도를 조향값으로 변환합니다.
        # 각도 검출 범위 = (-0.5*pi , 0.5*pi) radian (왼쪽, 오른쪽) 
        # 아래 코드는 (-0.5*pi, 22), (0.5*pi, -15) 점들을 지나는 직선 방정식을 표현함
        # 각도 검출 범위는 바뀌지 않을테니 유사시 max,min 값들을 조정하여 직선 방정식을 새로 산출할 것

        steering = -(37 / (0.5*pi))*(angle - 0.25*pi) - 15

        steering = int(max(min(steering, self.max_steering), self.min_steering))
        return steering  # Convert to integer

    def run(self):
        rospy.spin()

def image_callback(msg):
    global frame
    bridge = CvBridge()
    frame = bridge.compressed_imgmsg_to_cv2(msg, "bgr8")  # Change Image_msg to image_array(cv2)

def Steer_publisher():
    global steering_value
    pub = rospy.Publisher('steer_value', Int16, queue_size=10) # define topic
    rospy.Subscriber('/usb_cam/image_raw/compressed', CompressedImage,image_callback)

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        if frame is not None:
            # Use the calculated steering value from DecisionMaker
            if steering_value is not None:
                steer_msg = Int16()
                steer_msg.data = steering_value
                pub.publish(steer_msg)

                rospy.loginfo(f'steering value: {steer_msg.data}')

        rate.sleep()

if __name__ == "__main__":
    try:
        # Initialize and run the DecisionMaker
        decision_maker = DecisionMaker()

        # Start the Steer_publisher function in a separate thread
        steer_thread = threading.Thread(target=Steer_publisher)
        steer_thread.start()

        # Start the DecisionMaker
        decision_maker.run()
    except rospy.ROSInterruptException:
        pass
