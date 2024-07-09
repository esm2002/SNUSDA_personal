#!/usr/bin/env python3

'''
test_traffic_decision.py

1. 라이다, 카메라에서 받은 정보를 바탕으로 신호인식 & 정지, 차선변경, 자율주행을 판단하는 코드
2. lidar_lanechange_flag.py에서 publish한 topic(obstacle_flag, y_coord)를 subscribe
3. lane_masking_re.py에서 publish한 topic(lane_detect)를 subscribe
4. traffic_light_color_decoder_final.py에서 publish한 topic(stop_flag, bbox_size)를 subscribe
5. 아두이노(serial_node)에 최종 명령 publish

'''
import rospy
import rosnode
from std_msgs.msg import Int16
from std_msgs.msg import Int32
from std_msgs.msg import Float32
from lidar.msg import obstacle_detection # obstacle_detection.msg 파일 참고(custom msg) : flag, y_coord
from camera.msg import traffic_light_stop # traffic_light_stop.msg 파일 참고(custom msg) : stop_flag, bbox_size

from queue import Queue
import time

class ControlCommand() : 
    
    def __init__(self) : 
        
        rospy.init_node('decision', anonymous=True) # decision라는 이름으로 노드 초기화

        rospy.Subscriber('obstacle_flag', obstacle_detection, self.lidar_callback) # lidar_lanechage_flag.py 에서 stop_flag, y_coord를 받아옴
        rospy.Subscriber('lane_detect', Float32, self.lane_masking_callback) # lane_masking_re.py에서 조향값을 받아옴
        rospy.Subscriber('stop_signal', traffic_light_stop, self.traffic_ligit_detect_callback) # traffic_light_color_decoder_final.py에서 stop_flag, bbox_size를 받아옴
        
        # rospy.Subscriber('crosswalk_distance', Int32, self.crosswalk_detect_callback) # crosswalk_detection_node_final.py에서 crosswalk_distance를 받아옴 / 현재 코드에서는 사용하지 않음. 
        
        self.lidar_obstacle_detect = 0 # 장애물 감지 여부
        self.lidar_obstacle_y_coord = 0 # 장애물의 y좌표

        self.lane_masking = 0 # 조향값
        self.received_msgs_lane_maksing = Queue(maxsize=3) # lane_masking_re.py에서 받아온 조향값을 3개 저장하는 queue
        
        self.traffic_ligit_detect_stop = 0 # 신호등 정지 여부
        self.traffic_ligit_detect_size = 0 # 신호등 bbox_size
        self.crosswalk_detect = 0 # 횡단보도 감지 여부

        self.cn = 0
        


    def lidar_callback(self, msg) : # lidar_lanechage_flag.py 에서 stop_flag, y_coord를 받아오는 callback 함수
       
        self.lidar_obstacle_detect = msg.flag
        self.lidar_obstacle_y_coord = msg.y_coord
        
        # 아래 주석은 장애물이 1회 감지 시 lidar 관련 노드를 종료하는 내용

        # if(self.lidar_obstacle_detect == 1) :
        #     print("장애물이 감지되어 lidar 관련 노드를 종료합니다..")
            
        #     rosnode.kill_nodes(['obstacle_detection'])
        #     rosnode.kill_nodes(['cluster_pub'])
            
    def lane_masking_callback(self, msg) : # lane_masking_re.py에서 조향값을 받아 최종 조향값을 계산하는 callback 함수
        
        if(self.received_msgs_lane_maksing.full()): # queue가 꽉 차있으면
            self.received_msgs_lane_maksing.get() # queue에서 조향값 1개 뺌
        
        self.received_msgs_lane_maksing.put(msg.data) # queue에 조향값을 넣음
        
        if self.received_msgs_lane_maksing.empty(): # queue가 비어있으면
            self.lane_masking = 0 # 최종 조향값을 0으로 설정
            
        else :
            self.lane_masking =  round(sum(self.received_msgs_lane_maksing.queue) / self.received_msgs_lane_maksing.qsize()) # queue에 있는 조향값의 평균을 최종 조향값으로 사용
         
    def traffic_ligit_detect_callback(self, msg) : # traffic_light_color_decoder_final.py에서 stop_flag, bbox_size를 받아오는 callback 함수
        
        self.traffic_ligit_detect_stop = msg.stop_flag
        self.traffic_ligit_detect_size = msg.bbox_size
        
    def crosswalk_detect_callback(self, msg) : # crosswalk_detection_node_final.py에서 crosswalk_distance를 받아오는 callback 함수, 현재는 사용하지 않음
        
        self.crosswalk_detect = msg.data
        
    def arduino_command_pub(self): # 아두이노에 최종 명령 publish
        
        self.command_pub = rospy.Publisher('control_command_pub', Int16, queue_size=3) # 아두이노에 최종 명령 publish하는 publisher
        
        control_msg = Int16()
        
        distance_from_cross_walk = 9 # 횡단보도와의 기준 거리
        bbox_size = 15500 # 신호등 bbox_size 기준


         ###################### 신호등 인식 및 정지 ##################################
        if(self.traffic_ligit_detect_stop == 1 and self.lidar_obstacle_detect == 0) : # 신호등 stop && 장애물 미감지
            
            print("bbox size : ", self.traffic_ligit_detect_size, "distance_from_cross_walk : ", self.crosswalk_detect) # for debugging
            
            if(self.traffic_ligit_detect_size >= bbox_size ): # 신호등 bbox_size가 기준보다 크면
                control_msg.data = 48 # 정지명령
                self.command_pub.publish(control_msg)
                print("신호등이 감지되었습니다. 정지합니다.")
                self.cn = 1 # 신호등이 감지되어 정지한 적이 있음.
                rospy.sleep(1)

            else : # 신호등 bbox_size가 기준보다 작으면

                control_msg.data = self.lane_masking # 차선인식으로 얻은 조향값을 사용
                self.command_pub.publish(control_msg)
                print("신호등이 감지되었지만, 아직 멀어서 자율주행 중..") 
        
        ###################### 장애물 + Lane Change ##################################
        if(self.lidar_obstacle_detect == 1 and self.traffic_ligit_detect_stop == 0) : # 장애물 감지 && 신호등 go
            
            self.lidar_obstacle_detect = 0 # 장애물 감지 여부 초기화

            # 차선변경 룰베이스
            print("장애물이 감지되었습니다. 룰베이스 차선변경입니다.")
            y_coord = self.lidar_obstacle_y_coord # 차량이 틀어진 정도
            time_left = -1.77 # FIXME: 틀어진 정도에 따라 얼마나 더 좌회전해야할 지 결정하는 변수
            time_right = 0.035
            
            
            time_right_tilt = 3.3 # 우회전 시간
            time_left_tilt = 3.3+y_coord*time_left # 기준 좌회전 시간에 y좌표에 따라 추가 시간
            time_straight = 0.5 # 직진 시간

            control_msg.data = 48 # 정지 명령

            print("0. 일단 정지")
            print('angle: ', y_coord) # 차량이 틀려있는 정도
            start_time = rospy.get_time()
            while(rospy.get_time()-start_time < 1) : # 1초만큼 정지
                time.sleep(0.1)
                self.command_pub.publish(control_msg)


            control_msg.data = 14 # 좌회전
            print("1. 좌회전", time_left_tilt)
            print(control_msg.data) # for debugging, 해당 명령 pub 시간 출력
            start_time = rospy.get_time()
            while(rospy.get_time()-start_time < time_left_tilt) : # 좌회전 시간만큼 명령 publish
                time.sleep(0.1)
                self.command_pub.publish(control_msg)


            control_msg.data =  -12 # 우회전
            print("2. 우회전")
            print(control_msg.data)
            start_time = rospy.get_time()
            while(rospy.get_time()-start_time < time_right_tilt) : # 우회전 시간만큼 명령 publish
                time.sleep(0.1)
                self.command_pub.publish(control_msg)
                
            
            control_msg.data = 0 # 직진
            print("3. 직진")
            start_time = rospy.get_time()
            while(rospy.get_time()-start_time < time_straight) : # 직진 시간만큼 명령 publish
                time.sleep(0.1)
                self.command_pub.publish(control_msg)
        

            control_msg.data =  -12 # 우회전
            print("4. 우회전")  
            print(control_msg.data)  
            start_time = rospy.get_time()
            while(rospy.get_time()-start_time < time_right_tilt-0.05) : # 우회전 시간만큼 명령 publish, 기준 시간에서 -0.05하여 보정
                time.sleep(0.1)
                self.command_pub.publish(control_msg)
               

            control_msg.data = 14 # 좌회전
            print("5. 좌회전")
            print(control_msg.data)
            start_time = rospy.get_time()
            while(rospy.get_time()-start_time < time_left_tilt-0.85) : # 좌회전 시간만큼 명령 publish, 기준 시간에서 -0.85하여 보정
                time.sleep(0.1)
                self.command_pub.publish(control_msg)
              

        ########################## 차선인식으로 자율주행 ###########################
        if(self.lidar_obstacle_detect == 0 and self.traffic_ligit_detect_stop == 0) : # lane change 끝 && 신호등 go
            
            # if(self.cn == 1) : # 신호등이 감지되어 정지한 적이 있으면 직진 명령을 5초간 publish, 대회 상황에서만 적용 가능
            #     control_msg.data = 0
            #     print("TL_green...")
            #     start_time = rospy.get_time()
            #     while(rospy.get_time()-start_time < 5) : 
            #         self.command_pub.publish(control_msg)
            #     self.cn = 0

            
            control_msg.data = self.lane_masking
            self.command_pub.publish(control_msg)
            print("자율주행 중입니다. ", control_msg.data)
            
        print(self.lidar_obstacle_detect, self.lane_masking) # for debugging, 장애물 감지 여부 및 조향값 출력


    def run(self) : 
        
        rate = rospy.Rate(10) # FIXME : check the rate
        
        while not rospy.is_shutdown():
                
           self.arduino_command_pub() # 아두이노에 최종 명령 publish하는 arduino_command_pub 함수 호출
           rate.sleep()


if __name__ == '__main__':
    
    try:
        Control_Command = ControlCommand() 
        Control_Command.run() # run 함수 호출
        
    except rospy.ROSInterruptException:
        pass