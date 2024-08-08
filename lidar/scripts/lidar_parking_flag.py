#!/usr/bin/env python3

import rospy
import rosnode
from geometry_msgs.msg import Polygon
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Int16
#from lidar.msg import parking_detection 
import numpy as np

'''
lidar_cluster_pub에서 발행한 cluster_coordinate와 cluster_point_nums를 받아서
cluster_coordinate의 x,y좌표와 cluster_point_nums의 point number를 이용하여
장애물 유무를 감지하는 코드
'''

class ParkingDetection:
    # 클래스 초기화 함수, 변수 설정
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node("parking_lidar") # 노드 이름 설정
        
        # Create a subscriber for the cluster coordinates and point numbers
        rospy.Subscriber('cluster_coordinate', Polygon, self.cluster_coord_callback) # Subscribe to the "cluster_coordinate" topic
        rospy.Subscriber('cluster_point_nums', Float64MultiArray, self.cluster_point_nums_callback) # Subscribe to the "cluster_point_nums" topic

        # Create a publisher for the obstacle flag
        self.parking_flag_pub = rospy.Publisher('parking_flag', Int16, queue_size=10)

        # Define variables for storing cluster coordinates and point numbers
        self.cluster_coordinates = []
        self.cluster_point_nums = []

        # Define variables for tracking obstacle detection
        self.count = 0 # Number of consecutive frames in which an obstacle is detected
        self.parking_flag = 0 # Flag indicating whether an obstacle is detected
        self.y_coord = 0 # y-coordinate of the obstacle


    def cluster_coord_callback(self, msg):
        # Process the cluster coordinates
        self.cluster_coordinates = [(point.x, point.y) for point in msg.points]
        self.check_obstacle()

    def cluster_point_nums_callback(self, msg):
        # Process the cluster point numbers
        self.cluster_point_nums = msg.data
        self.check_obstacle()

    def check_obstacle(self):
        # Check if the specified conditions for obstacle sdetection are met
        
        # Check if the number of cluster coordinates and cluster point numbers are equal
        if len(self.cluster_coordinates) == len(self.cluster_point_nums): # (np.sqrt(x**2+y**2) <= 4 and  x>0 and y>0 and 
            obstacle_detected = any(((-1)<x and x<0 and 0<y and y<3 and  num >= 50) for (x, y), num in zip(self.cluster_coordinates, self.cluster_point_nums))
        else:
            obstacle_detected = False

        # If the conditions are met, increment the count and set the obstacle flag
        if obstacle_detected:
            self.count += 1
            # If the count exceeds 5, set the obstacle flag (5번 연속으로 장애물이 감지되면 장애물이 감지된 것으로 판단)
            # 5는 변경 가능, 0으로 두면 인식되자 마자 flag가 1로 바뀜
            if self.count > 10: 
                self.parking_flag = 1
                print("parking_flag is on")
                # Extract indices of elements that satisfy the obstacle_detected condition
                # 1 <= np.sqrt(value[0]**2+value[1]**2) <= 2.1
                indices = [index for index, value in enumerate(self.cluster_coordinates) if ((-1)<value[0] and value[0]<0 and 0<value[1] and value[1]<3 and  self.cluster_point_nums[index] >= 10)]

                # If there are indices that satisfy the condition, find the index with the maximum y-coordinate
                if indices:
                    max_y_index = max(indices, key=lambda index: self.cluster_coordinates[index][1])
                    #self.y_coord = np.arctan(self.cluster_coordinates[max_y_index][1]/self.cluster_coordinates[max_y_index][0])*180/np.pi
                    self.y_coord = self.cluster_coordinates[max_y_index][1]
                    print(self.y_coord)

        # If the conditions are not met, reset the count and obstacle flag            
        elif not obstacle_detected:
            # Reset the count and obstacle flag
            self.count = 0
            self.parking_flag = 0
            self.y_coord = 0

    def publish_obstacle_flag(self):
        # Publish the obstacle flag
        parking_flag_msg = Int16()
        parking_flag_msg.data = self.parking_flag
      
    
        self.parking_flag_pub.publish(parking_flag_msg)

def main():
    parking_lidar = ParkingDetection()
    rate = rospy.Rate(10)  # 30 Hz (adjust the rate as needed)
    while not rospy.is_shutdown():
        parking_lidar.publish_obstacle_flag()
        rate.sleep()

if __name__ == "__main__":
    main()
