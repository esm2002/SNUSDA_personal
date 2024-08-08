#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int16
from pynput import keyboard

def on_press(key):
    try:
        if key.char == 'w':  # 전진
            rospy.loginfo("Key 'w' pressed")
            pub.publish(1)
        elif key.char == 'a':  # 좌회전
            rospy.loginfo("Key 'a' pressed")
            pub.publish(2)
        elif key.char == 'd':  # 우회전
            rospy.loginfo("Key 'd' pressed")
            pub.publish(3)
        elif key.char == 's':  # 정지
            rospy.loginfo("Key 's' pressed")
            pub.publish(4)
    except AttributeError:
        pass

def on_release(key):
    if key == keyboard.Key.esc:
        return False

if __name__ == '__main__':
    rospy.init_node('keyboard_control')
    pub = rospy.Publisher('steering_command', Int16, queue_size=10)
    
    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()
    
    rospy.spin()
