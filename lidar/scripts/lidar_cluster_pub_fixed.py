#!/usr/bin/env python3

#모듈 가져오기
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point, Polygon
from std_msgs.msg import Float64MultiArray
from math import *
import os
import numpy as np
from sklearn.cluster import DBSCAN
import time

'''
lidar_cluster_pub.py
    - 라이다 raw data를 극 좌표에서 직교 좌표로 변환 후, 거리에 따라 군집화하여 토픽 발행
    - 거리 범위에 따라 군집화를 다르게 하여, 근거리, 중거리, 원거리 장애물을 구분
    - 군집 중심 좌표 및 군집 내 포인트 개수를 토픽으로 발행
    - lidar_cluster_sub_rviz.py에 보내 RViz에 띄워주는 역할
    - lidar_lanechange_flag.py에 보내 장애물 존재 여부 및 차선 변경 여부 판단에 사용됨
'''

#클래스 생성
class Lidar_ctrl:

    # 초기화 함수, 변수 설정
    def __init__(self):
        #노드 이름 설정
        rospy.init_node("cluster_pub")
        #노드 역할 설정
        self.coord_pub = rospy.Publisher('cluster_coordinate', Polygon, queue_size=10) # 토픽 이름은 cluster_coordinate, 메시지 형식은 Polygon, 군집화 중심 좌표를 발행
        self.num_pub = rospy.Publisher('cluster_point_nums', Float64MultiArray, queue_size=10) # 토픽 이름은 cluster_point_nums, 메시지 형식은 Float64MultiArray, 동일 군집 내 포인트 개수를 발행
        rospy.Subscriber("/scan",LaserScan,self.lidar_CB) # RPLidar의 raw data를 받아두어 lidar_CB 함수 실행
        
        self.lidar_msg = LaserScan()
        self.DMAX = 7000  # 최대 거리
        self.MIN_ANGLE = 90 # 최소 각도
        self.MAX_ANGLE = -90 # 최대 각도
        self.QUALITY = 5 # 최소 품질

        # 거리에 따른 군집화를 위한 변수 설정, 변경 가능
        self.threshold_short = 600 # 근거리 장애물 거리 범위 (mm), 0~600mm
        self.threshold_mid = 3000 # 중거리 장애물 거리 범위 (mm), 600~3000mm
        self.threshold_far = 5000 # 원거리 장애물 거리 범위 (mm), 3000~5000mm

        # 거리에 따른 군집화 시, DBSCAN의 eps, n 값 설정, 변경 가능
        self.eps_short, self.eps_mid, self.eps_far = 100, 150, 200
        self.n_short, self.n_mid, self.n_far = 50, 30, 20

    # 군집화 중심 좌표와 동일 군집 내 포인트 개수를 토픽으로 발행
    def publish_coordinates_and_num(self, coordinates_list, point_num_msg):  
        polygon_msg = Polygon() # Polygon 메시지 형식으로 polygon_msg 변수 생성
        point_num = Float64MultiArray() # Float64MultiArray 메시지 형식으로 point_num 변수 생성
        point_num.data = point_num_msg
        if len(coordinates_list)>0:
            for cord in coordinates_list:
                point_msg = Point()
                point_msg.x, point_msg.y, point_msg.z = cord[0], cord[1], 0
                polygon_msg.points.append(point_msg)
        self.coord_pub.publish(polygon_msg)
        self.num_pub.publish(point_num)

    # x, y좌표와 eps, n값을 입력받아 DBSCAN을 실행하고, 군집화된 결과를 반환
    def clustering(self, x, y, eps=80, n=10):
        dbscan = DBSCAN(eps=eps, min_samples=n)
        coordinates = np.column_stack((x, y))
        clusters = dbscan.fit_predict(coordinates)
        clusters = np.array(clusters)
        # print(coordinates)
        return clusters, coordinates

    # 극 좌표를 직교 좌표로 변환
    def polar_to_cartesian(self, angle, num):
        x = num * np.cos(np.radians(angle))
        y = num * np.sin(np.radians(angle))
        return x, y

    # 데이터 필터링 및 거리에 따른 데이터 분리
    """
    def separate_points_3(self, qualities, nums, angles, min_qualities=7, threshold_short=600, threshold_mid=1500, threshold_max=3000):
        # 근거리: 데이터 품질이 min_qualities 이상이고, 거리가 threshold_short 이하인 데이터만 사용
        filtered_indices_short = np.where((qualities >= min_qualities) & (nums < threshold_short) \
                                          & ((self.MIN_ANGLE <= angles) | (angles <= self.MAX_ANGLE)))[0]
        filtered_angles_short = angles[filtered_indices_short]
        filtered_nums_short = nums[filtered_indices_short]

        # 중거리: 데이터 품질이 min_qualities 이상이고, 거리가 threshold_mid 이하인 데이터만 사용
        filtered_indices_mid = np.where((qualities >= min_qualities) & (threshold_short<= nums) & (nums<= threshold_mid) & \
                                        ((self.MIN_ANGLE <= angles) | (angles <= self.MAX_ANGLE)))[0]
        filtered_angles_mid = angles[filtered_indices_mid]
        filtered_nums_mid = nums[filtered_indices_mid]

        # 원거리: 데이터 품질이 min_qualities 이상이고, 거리가 threshold_max 이하인 데이터만 사용
        filtered_indices_far = np.where((qualities >= min_qualities) & (threshold_mid<= nums) & (nums<=threshold_max) & \
                                        ((self.MIN_ANGLE <= angles) | (angles <= self.MAX_ANGLE)))[0]
        filtered_angles_far = angles[filtered_indices_far]
        filtered_nums_far = nums[filtered_indices_far]

        print('short point number: ', len(filtered_angles_short))
        print('mid point number: ', len(filtered_angles_mid))
        print('far point number: ', len(filtered_angles_far))

        return filtered_angles_short, filtered_nums_short, filtered_angles_mid, filtered_nums_mid, filtered_angles_far, filtered_nums_far"""
        
    #각도 기준으로 분류하는 코드 추가
    def separate_points_by_angle(self, qualities, nums, angles, min_qualities=7, min_angle=-20, max_angle=20, threshold_short=600, threshold_mid=1500, threshold_max=3000 ):
        # 근거리: 데이터 품질이 min_qualities 이상이고, 거리가 threshold_short 이하인 데이터만 사용
        filtered_indices_short = np.where((qualities >= min_qualities) & (nums < threshold_short) & (angles >= min_angle) & (angles <= max_angle))[0]
        filtered_angles_short = angles[filtered_indices_short]
        filtered_nums_short = nums[filtered_indices_short]
        
        # 중거리: 데이터 품질이 min_qualities 이상이고, 거리가 threshold_mid 이하인 데이터만 사용
        filtered_indices_mid = np.where((qualities >= min_qualities) & (nums >= threshold_short) & (nums < threshold_mid) & (angles >= min_angle) & (angles <= max_angle))[0]
        filtered_angles_mid = angles[filtered_indices_mid]
        filtered_nums_mid = nums[filtered_indices_mid]
        
        # 원거리: 데이터 품질이 min_qualities 이상이고, 거리가 threshold_max 이하인 데이터만 사용
        filtered_indices_far = np.where((qualities >= min_qualities) & (nums >= threshold_mid) & (nums < threshold_max) & (angles >= min_angle) & (angles <= max_angle))[0]
        filtered_angles_far = angles[filtered_indices_far]
        filtered_nums_far = nums[filtered_indices_far]
        
        return filtered_angles_short, filtered_nums_short, filtered_angles_mid, filtered_nums_mid, filtered_angles_far, filtered_nums_far


    # 군집 중심 좌표와 동일 군집 내 포인트 개수를 계산, 리스트로 반환
    def calculate_cluster_nums(self, clusters, offsets):
        cluster_centers = []
        cluster_nums = []

        unique_clusters = np.unique(clusters[clusters >= 0])
        for cluster in unique_clusters:
            cluster_points = offsets[clusters == cluster]
            cluster_center = np.mean(cluster_points, axis=0)
            cluster_num = len(cluster_points)
            cluster_centers.append(cluster_center)
            cluster_nums.append(cluster_num)

        print(cluster_centers)

        return unique_clusters, cluster_centers, cluster_nums

    # 거리에 따른 점 분리 후, 군집화 실행
    def range_clustering(self, x_short, y_short, x_mid, y_mid, x_far, y_far, eps_short=80, eps_mid=100, eps_far=200):
        clusters_short, offsets_short = [], []
        clusters_mid, offsets_mid = [], []
        clusters_far, offsets_far = [], []
        unique_clusters_short, unique_clusters_mid, unique_clusters_far = [], [], []
        center_group = []
        num_group = []

        if len(x_short) > 0:
            clusters_short, offsets_short = self.clustering(x_short, y_short, eps=eps_short, n=self.n_short)
        if len(x_mid) > 0:    
            clusters_mid, offsets_mid = self.clustering(x_mid, y_mid, eps=eps_mid, n=self.n_mid)
        if len(x_far) > 0:     
            clusters_far, offsets_far = self.clustering(x_far, y_far, eps=eps_far, n=self.n_far)

        if len(clusters_short) > 0:
            unique_clusters_short, cluster_centers_short, cluster_nums_short = self.calculate_cluster_nums(clusters_short, offsets_short)
            for cluster, num, center in zip(unique_clusters_short, cluster_nums_short, cluster_centers_short):
                # print(f"num: {num:.2f} (short) & Center: ({center[0]:.2f}, {center[1]:.2f})")
                center_group.append(center*0.001)
                num_group.append(num)

        if len(clusters_mid) > 0:
            unique_clusters_mid, cluster_centers_mid, cluster_nums_mid = self.calculate_cluster_nums(clusters_mid, offsets_mid)
            for cluster, num, center in zip(unique_clusters_mid, cluster_nums_mid, cluster_centers_mid):
                # print(f"num: {num:.2f} (mid) & Center: ({center[0]:.2f}, {center[1]:.2f})")
                center_group.append(center*0.001)
                num_group.append(num)

        if len(clusters_far) > 0:
            unique_clusters_far, cluster_centers_far, cluster_nums_far = self.calculate_cluster_nums(clusters_far, offsets_far)
            for cluster, num, center in zip(unique_clusters_far, cluster_nums_far, cluster_centers_far):
                # print(f"num: {num:.2f} (far) & Center: ({center[0]:.2f}, {center[1]:.2f})")
                center_group.append(center*0.001)
                num_group.append(num)
        
        return  center_group, num_group

    #함수 설정
    def lidar_CB(self,msg):

        self.lidar_msg = msg
        os.system('clear')
        start_time = time.time()   # 시간 측정 시작

        print(f"range: {self.threshold_short, self.threshold_mid, self.threshold_far}")
        print(f"eps:{self.eps_short, self.eps_mid, self.eps_far}")
        print(f"n:{self.n_short, self.n_mid, self.n_far}")
        # print(f"angle_min : {msg.angle_min}") #radian
        # print(f"quality: {msg.intensities}")
        # print('angles: ', angles)
        # print(msg.range_min, msg.range_max)
        
        # 라이다 측정값을 각도, 거리, 품질로 분리
        angles = np.array([(msg.angle_min + msg.angle_increment*index)*180/pi for index, value in enumerate(msg.ranges)])
        nums = np.array([point_num*1000 for point_num in msg.ranges])
        qualities = np.array(msg.intensities)

        print(len(angles))
        #print(len(nums))
        
        # 데이터 필터링 및 거리에 따른 데이터 분리
        # 데이터 품질이 40 이상인 데이터만 사용, 거리에 따라 근거리(threshold_short), 중거리(threshold_mid), 원거리(threshold_max)로 분리
        filtered_angles_short, filtered_nums_short, filtered_angles_mid, filtered_nums_mid, \
            filtered_angles_far, filtered_nums_far \
                = self.separate_points_by_angle(qualities, nums, angles, min_qualities=40, -20,20,
                                         threshold_short=self.threshold_short, threshold_mid=self.threshold_mid, threshold_max=self.threshold_far)    

        # 극 좌표를 직교 좌표로 변환
        x_short, y_short = self.polar_to_cartesian(filtered_angles_short, filtered_nums_short)
        x_mid, y_mid = self.polar_to_cartesian(filtered_angles_mid, filtered_nums_mid)
        x_far, y_far = self.polar_to_cartesian(filtered_angles_far, filtered_nums_far)

        # 거리에 따른 군집화
        center_group, num_group = self.range_clustering(x_short, y_short, x_mid, y_mid, x_far, y_far, eps_short=self.eps_short, eps_mid=self.eps_mid, eps_far=self.eps_far)
        # 군집화 결과 토픽 발행
        self.publish_coordinates_and_num(center_group, num_group)
        print('center_group', center_group)
        print('num_group', num_group)
        elapsed_time = time.time() - start_time # 시간 측정 종료
        print((f"Time: {elapsed_time:.10f}"))

def main():
    lidar_ctrl = Lidar_ctrl()
    rospy.spin()

if __name__ == "__main__" :
    main()
