#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Point, Polygon
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA

'''
lidar_cluster_sub_rviz.py
    - 라이다 클러스터링 결과를 RViz에 띄우기 위한 노드
    - lidar_cluster_pub.py 노드 실행 후 실행, cluster_coordinate 토픽 구독
    - 실차 주행 시에는 사용되지 않고 테스트용으로 사용됨(시각화, 변수 조정 등)
    - 라이다 클러스터링 결과를 구독하고, RViz에 띄우기 위해 visualization_msgs/Marker 메시지를 발행 
'''


def callback(data):
    # RViz에 좌표를 띄우기 위해 visualization_msgs/Marker 메시지 사용
    marker = Marker() # Marker 메시지 형식으로 marker 변수 생성
    marker.header.frame_id = "laser"  # RViz에서 보여줄 좌표의 좌표계 설정 (RViz에서 이름은 laser로 바꿔줘야 함)
    marker.type = Marker.POINTS # 마커의 모양 설정 (점 형태)
    marker.action = Marker.ADD 
    marker.scale.x = 0.05 # 마커의 크기 설정 (변경하지 않아도 됨)
    marker.scale.y = 0.05 # 마커의 크기 설정 (변경하지 않아도 됨)
    marker.color = ColorRGBA(0.0, 0.0, 1.0, 1.0)  # 색상 (파란색)
    marker.lifetime = rospy.Duration(1)  # RViz에서 띄워진 좌표가 유지될 시간 (초)

    for point in data.points:
        marker.points.append(point)

    # 마커를 발행하기 위해 발행자 생성
    marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10) # 토픽 이름은 visualization_marker
    marker_pub.publish(marker) 

def polygon_subscriber():
    # 노드 초기화 및 토픽 구독자 생성
    rospy.init_node('rviz_subscriber', anonymous=True)
    rospy.Subscriber('cluster_coordinate', Polygon, callback) # cluster_coordinate 토픽 구독 후 callback 함수 실행
    
    rospy.spin()

if __name__ == '__main__':
    try:
        polygon_subscriber()
    except rospy.ROSInterruptException:
        pass