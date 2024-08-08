#!/usr/bin/env python3

import rospy
import rosnode
from std_msgs.msg import Int16
from std_msgs.msg import Int32
from std_msgs.msg import Float32
from control.msg import control_command
from lidar.msg import obstacle_detection
from camera.msg import traffic_light_stop

from queue import Queue


class ControlCommand() : 
    
    def __init__(self) : 
        
        rospy.init_node('decision', anonymous=True)

        rospy.Subscriber('Steering_value', Int16, self.End_to_steer_callback)
        
        self.Steer_value = Int16()
        
            
    
    def End_to_steer_callback(self, msg):
        
        self.Steer_value = msg.data
        
    def arduino_command_pub(self):
        
        self.command_pub = rospy.Publisher('final_val', Int16, queue_size=3)
        
        control_msg = Int16()
        
       
            

        # ########################### End_to_Steer_Ver ###########################
            # lane change 끝 && 신호등 go
            

        #self.command_pub.publish(control_msg)
        print("자율주행 중입니다. ", self.Steer_value)

        control_msg.data = self.Steer_value


        self.command_pub.publish(control_msg)

        
    def run(self) : 
        
        # rate = rospy.Rate(10)
        rate = rospy.Rate(30)
        
        while not rospy.is_shutdown():
                
           self.arduino_command_pub() 
           rate.sleep()


if __name__ == '__main__':
    try:
        Control_Command = ControlCommand()
        Control_Command.run()
        
    except rospy.ROSInterruptException:
        pass