#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int16

def keyboard_publisher():
    # ROS 노드 초기화
    rospy.init_node('keyboard_publisher', anonymous=True)

    # 'keyboard_input' 토픽에 메시지를 게시하는 Publisher 생성
    pub = rospy.Publisher('keyboard_input', Int16, queue_size=10)

    # 1Hz로 루프를 실행하면서 키보드 입력을 받아서 게시
    rate = rospy.Rate(1)
    msg = Int16()
    print("1 : left / 2 : front / 3 : right / other : stop")
    
    while not rospy.is_shutdown():
        # 키보드 입력 받기
        
        msg.data = int(input("Enter a keyboard message: "))
        print()
        pub.publish(msg)

        # 루프 지연
        rate.sleep()

if __name__ == '__main__':
    try:
        keyboard_publisher()
    except rospy.ROSInterruptException:
        pass
