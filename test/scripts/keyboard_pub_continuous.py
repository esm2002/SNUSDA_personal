#!/usr/bin/env python3

import rospy
import threading
from std_msgs.msg import Int16

command = 0
new_input = 0
courrent_input = 0

def publish_input():
    
    global command
    
    rospy.init_node('keyboard_input_publisher', anonymous=True)
    pub = rospy.Publisher('keyboard_input', Int16, queue_size=10)

    rate = rospy.Rate(10)  # 10 Hz publish rate
    msg = Int16()
    
    while not rospy.is_shutdown():
        msg.data = command
        pub.publish(msg)
        rate.sleep() 

def keyboard_input():
    
    global command
    
    while not rospy.is_shutdown():
        
        command = int(input("Enter an integer: "))
        
        
    
def main() : 
    
    global command

    keyboard_thread = threading.Thread(target=keyboard_input) # publisher thread 선언
    keyboard_thread.daemon = True
    keyboard_thread.start()
    
    publish_input()
    

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    
