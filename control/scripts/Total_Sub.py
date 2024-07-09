#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int16
from std_msgs.msg import Int32
from std_msgs.msg import Float32
from control.msg import control_command
from lidar.msg import obstacle_detection

from queue import Queue


class ControlCommand() : 
    
    def __init__(self) : 
        
        rospy.init_node('decision', anonymous=True)

        rospy.Subscriber('obstacle_flag', obstacle_detection, self.lidar_callback)
        rospy.Subscriber('lane_detect', Float32, self.lane_masking_callback)
        rospy.Subscriber('/vision/stop_signal', Int32, self.traffic_ligit_detect_callback) # 신호등 sub
        rospy.Subscriber('/vision/crosswalk_distance', Int32, self.crosswalk_detect_callback) # 횡단보도 sub
        rospy.Subscriber('Steer_value', Int16, self.End_to_steer_callback)
        
        self.lidar_obstacle_detect = 0
        self.lidar_obstacle_y_coord = 0
        self.lane_masking = 0
        self.Steer_value = Int16()

        self.received_msgs_lane_maksing = Queue(maxsize=3)
        
        self.traffic_ligit_detect = 0
        self.crosswalk_detect = 0


    def lidar_callback(self, msg) : # lidar callback
        #rospy.loginfo("Received value from subscriber1: %f", msg.data)
        self.lidar_obstacle_detect = msg.flag
        self.lidar_obstacle_y_coord = msg.y_coord

    def lane_masking_callback(self, msg) : 
        
        #print(msg.data)
        
        if(self.received_msgs_lane_maksing.full()):
            self.received_msgs_lane_maksing.get()
        
        self.received_msgs_lane_maksing.put(msg.data)
        
        if self.received_msgs_lane_maksing.empty():
            self.lane_masking = 0
            
        else :
            self.lane_masking =  round(sum(self.received_msgs_lane_maksing.queue) / self.received_msgs_lane_maksing.qsize())
    
    def End_to_steer_callback(self, msg):
        
        self.Steer_value = msg.data
        
    def traffic_ligit_detect_callback(self, msg) :
        
        self.traffic_ligit_detect = msg.data
        
    def crosswalk_detect_callback(self, msg) :
        
        self.crosswalk_detect = msg.data
        
    def arduino_command_pub(self):
        
        self.command_pub = rospy.Publisher('control_command_pub', Int16, queue_size=3)
        
        control_msg = Int16()
        
        a = 40
        
        if(self.traffic_ligit_detect == 1) : # 빨간불이면
    
            if(self.crosswalk_detect <= a) :
                control_msg.data = 48 # 정지해라
                #self.command_pub.publish(control_msg)
                print("신호등이 감지되었습니다. 정지합니다.")
                
            else :
                print("신호등이 감지되었지만, 아직 멀다.")
        
        elif(self.lidar_obstacle_detect == 1 and self.traffic_ligit_detect == 0) :
            # 장애물 인식 && 신호등 go
            
            # 차선변경 룰베이스
            print("장애물이 감지되었습니다. 룰베이스 차선변경입니다.")
            
            y_coord = self.lidar_obstacle_y_coord #좌
            time = 0.4
            change = 4
            
            control_msg.data = 48 # 정지
            self.command_pub.publish(control_msg)
            print("0. 일단 정지")
            rospy.sleep(1)
            
            
            control_msg.data = 17 # 좌회전
            self.command_pub.publish(control_msg)
            print("1. 좌회전", change-y_coord * time)
            rospy.sleep(change-y_coord * time )
            

            
            control_msg.data =  -17 # 우회전
            self.command_pub.publish(control_msg)
            print("2. 우회전")
            rospy.sleep(change)
            
            control_msg.data = 0 # 직진
            self.command_pub.publish(control_msg)
            print("3. 직진")
            rospy.sleep(1)

            control_msg.data =  -16 # 우회전
            self.command_pub.publish(control_msg)
            print("4. 우회전")            
            rospy.sleep(change-0.4)


            control_msg.data = 17 # 좌회전
            self.command_pub.publish(control_msg)
            print("5. 좌회전")
            rospy.sleep(change)


            control_msg.data = 0 # 직진
            self.command_pub.publish(control_msg)
            print("6. 직진")
            rospy.sleep(2)

        ########################### End_to_Steer_Ver ###########################
        elif(self.lidar_obstacle_detect == 0 and self.traffic_ligit_detect == 0) :
            # lane change 끝 && 신호등 go
            

            #self.command_pub.publish(control_msg)
            print("자율주행 중입니다. ", self.Steer_value)

            control_msg.data = self.Steer_value

        else :
            #self.command_pub.publish(control_msg)
            print("자율주행 중입니다.", self.Steer_value)
            
            control_msg.data = self.Steer_value

        self.command_pub.publish(control_msg)

        ########################### Lane Detection Ver ###########################
        # elif(self.lidar_obstacle_detect == 0 and self.traffic_ligit_detect == 0) :
        #     # lane change 끝 && 신호등 go
            
        #     control_msg.data = self.lane_masking
        #     #self.command_pub.publish(control_msg)
        #     print("자율주행 중입니다. ", control_msg.data)
        
        # else :
        #     control_msg.data = self.lane_masking
        #     #self.command_pub.publish(control_msg)
        #     print("자율주행 중입니다.")
            
        # print(self.lidar_obstacle_detect, self.lane_masking)
        
        # self.command_pub.publish(control_msg)

    def run(self) : 
        
        rate = rospy.Rate(5)
        
        while not rospy.is_shutdown():
                
           self.arduino_command_pub() 
           rate.sleep()


if __name__ == '__main__':
    try:
        Control_Command = ControlCommand()
        Control_Command.run()
        
    except rospy.ROSInterruptException:
        pass
