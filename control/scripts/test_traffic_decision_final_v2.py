#!/usr/bin/env python3

'''
test_traffic_decision.py

1. 라이다, 카메라에서 받은 정보를 바탕으로 신호인식 & 정지, 차선변경, 자율주행을 판단하는 코드
2. lidar_lanechange_flag.py에서 publish한 topic(obstacle_flag, y_coord)를 subscribe
3. lane_masking_re.py에서 publish한 topic(lane_detect)를 subscribe
4. traffic_light_color_decoder_final.py에서 publish한 topic(stop_flag, bbox_size)를 subscribe
5. 아두이노(serial_node)에 최종 명령 publish

+ 장애물 & lane change 부분 원래 코드에서 변경
!!!!!!!!신호등 인식 제외한 코드 -> 장애물 회피 테스트용 , 신호등 코드랑 합쳐야함
'''
import rospy
import rosnode
from std_msgs.msg import Int16
from std_msgs.msg import Int32
from std_msgs.msg import Float32

from std_msgs.msg import String
from lidar.msg import obstacle_detection # obstacle_detection.msg 파일 참고(custom msg) : flag, y_coord
from camera.msg import traffic_light_stop # traffic_light_stop.msg 파일 참고(custom msg) : stop_flag, bbox_size

from queue import Queue
import time

class ControlCommand() : 
    
    def __init__(self) :
        rospy.init_node('decision', anonymous=True) # decision라는 이름으로 노드 초기화
        rospy.Subscriber('distance',Int32,self.distance_callback) #run.ino에서 publish
        rospy.Subscriber('obstacle_detection',Int16,self.obstacle_callback)
        rospy.Subscriber('lane_detect', Float32, self.lane_masking_callback) # lane_masking_re.py에서 조향값을 받아옴
        rospy.Subscriber('stop_signal', traffic_light_stop, self.traffic_ligit_detect_callback) # traffic_light_color_decoder_final.py에서 stop_flag, bbox_size를 받아옴
        rospy.Subscriber('crosswalk_distance', Int32, self.crosswalk_detect_callback) # crosswalk_detection_node_final.py에서 crosswalk_distance를 받아옴
        
        self.lidar_obstacle_detect = 0 # 장애물 감지 여부
        self.front_distance = float('inf')  # 큰 값으로 초기화
        self.side_distance = float('inf')  # 큰 값으로 초기화
        self.command_pub = rospy.Publisher('control_command_pub', Int16, queue_size=3)
        
        self.lane_masking = 0 # 조향값
        self.received_msgs_lane_maksing = Queue(maxsize=3)
        self.ck=0

        self.traffic_ligit_detect_stop = 0 # 신호등 정지 여부
        self.traffic_ligit_detect_size = 0 # 신호등 bbox_size
        self.crosswalk_detect = 0 # 횡단보도 감지 여부
        self.cn = 0
        
    def distance_callback(self,msg):
    	self.front_distance=msg.data//10000
    	self.side_distance=msg.data%10000
    	
    def obstacle_callback(self,msg):
        self.lidar_obstacle_detect=msg.data
        
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
        
    def crosswalk_detect_callback(self, msg) : # crosswalk_detection_node_final.py에서 crosswalk_distance를 받아오는 callback 함수
        
        self.crosswalk_detect = msg.data
        
    def arduino_command_pub_1(self): # 아두이노에 최종 명령 publish
        control_msg = Int16()
        ###################### 장애물 + Lane Change ##################################
        if(self.lidar_obstacle_detect == 1) : # 장애물 감지
            self.lidar_obstacle_detect = 0 # 장애물 감지 여부 초기화
            # 차선변경 룰베이스
            print("장애물이 감지되었습니다.")
            
            time_right_tilt = 6.5 # 우회전 시간
            time_left_tilt = 7 # 좌회전 시간
            time_straight = 0.5 # 직진 시간

            # 0. 1초간 정지
            print('정지')
            control_msg.data = 48 # 정지 명령
            start_time = rospy.get_time()
            while(rospy.get_time()-start_time < 1) : # 1초만큼 정지
                time.sleep(0.1)
                self.command_pub.publish(control_msg)
               
            # 1. 좌회전
            print('좌회전')
            control_msg.data = 17 #self.first
            start_time = rospy.get_time()
            while(rospy.get_time()-start_time < time_left_tilt) : # 좌회전 시간만큼 명령 publish
                time.sleep(0.1)
                self.command_pub.publish(control_msg)


            # 2. 우회전 
            print('우회전')
            control_msg.data = -15 #self.second
            print(control_msg.data)
            start_time = rospy.get_time()
            while(rospy.get_time()-start_time < time_right_tilt) : # 우회전 시간만큼 명령 publish
                time.sleep(0.1)
                self.command_pub.publish(control_msg)
                
            # 3. 직진
            control_msg.data = 0 # 직진 -> 전방 & 사이드 센서 거리 작을떄(장애물 있을 때) 직진으로 변경
            while self.front_distance <20 or self.side_distance<20: #FIXME
                print("앞 거리: ",self.front_distance," 옆 거리: ", self.side_distance)
                time.sleep(0.1)
                self.command_pub.publish(control_msg)
            	
            # 4. 우회전
            control_msg.data = -15 #self.second
            print(control_msg.data)
            start_time = rospy.get_time()
            while(rospy.get_time()-start_time < time_right_tilt) : # 우회전 시간만큼 명령 publish
                time.sleep(0.1)
                self.command_pub.publish(control_msg)
            
            # 5. 좌회전
            control_msg.data = 17 #self.first
            print(control_msg.data) # for debugging, 해당 명령 pub 시간 출력
            start_time = rospy.get_time()
            while(rospy.get_time()-start_time < time_left_tilt) : # 좌회전 시간만큼 명령 publish
                time.sleep(0.1)
                self.command_pub.publish(control_msg)
        
        ########################## 차선인식으로 자율주행 ###########################
        else: # lane change 끝 && 신호등 go
            
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




    def arduino_command_pub_2(self):
        control_msg = Int16()
        
        distance_from_cross_walk = 9 # 횡단보도와의 기준 거리 NM
        bbox_size = 3400 # 신호등 bbox_size 기준

        ###################### 신호등 인식 및 정지 ##################################
        if(self.traffic_ligit_detect_stop == 1 and self.lidar_obstacle_detect == 0) : # 신호등 stop && 장애물 미감지
            
            print("bbox size : ", self.traffic_ligit_detect_size, "distance_from_cross_walk : ", self.crosswalk_detect) # for debugging
            
            if(self.traffic_ligit_detect_size >= bbox_size or self.crosswalk_distance <= distance_from_cross_walk ):  # red의 bbox_size가 기준보다 크거나, 횡단보도와의 거리가 기준 거리보다 작을 때
                control_msg.data = 48 # 정지명령
                self.command_pub.publish(control_msg)
                
                if(self.traffic_light_detect_size >= bbox_size):
                    print("신호등이 감지되었습니다. 정지합니다.")
                else:
                    printf("횡단보도가 가까워졌습니다. 정지합니다.")

                self.cn = 1 # 신호등이나 횡단보도가 감지되어 정지한 적이 있음.
                rospy.sleep(1)

            else : # 신호등 bbox_size가 기준보다 작으면

                control_msg.data = self.lane_masking # 차선인식으로 얻은 조향값을 사용
                self.command_pub.publish(control_msg)
                print("신호등이 감지되었지만, 아직 멀어서 자율주행 중..")
        
        
        ############################## 장애물 + Lane Change ###############################
        if(self.lidar_obstacle_detect == 1 and self.ck==0) : # 장애물 감지
            self.ck=1
            self.lidar_obstacle_detect = 0 # 장애물 감지 여부 초기화
            print("장애물이 감지되었습니다.")
            
            time_right_tilt = 6.5 # 우회전 시간
            time_left_tilt = 6.5 # 좌회전 시간
            time_straight = 3 # 직진 시간

            # 0. 1초간 정지
            print('정지')
            control_msg.data = 48 # 정지 명령
            start_time = rospy.get_time()
            while(rospy.get_time()-start_time < 1) : # 1초만큼 정지
                time.sleep(0.1)
                self.command_pub.publish(control_msg)
               
            # 1. 좌회전
            print('좌회전')
            control_msg.data = 22 #self.first
            start_time = rospy.get_time()
            cnt=0
            while(rospy.get_time()-start_time < time_left_tilt) : # 좌회전 시간만큼 명령 publish
                time.sleep(0.1)
                self.command_pub.publish(control_msg)
                
                
                """if self.front_distance < 40:
                    cnt=1
                if self.front_distance > 60 and cnt==1:
                    break"""
                    
            # 0. 1초간 정지
            print('정지')
            control_msg.data = 48 # 정지 명령
            start_time = rospy.get_time()
            while(rospy.get_time()-start_time < 1) : # 1초만큼 정지
                time.sleep(0.1)
                self.command_pub.publish(control_msg)


            # 2. 우회전 
            print('우회전')
            control_msg.data = -15 #self.second
            print(control_msg.data)
            start_time = rospy.get_time()
            while(rospy.get_time()-start_time < time_right_tilt) : # 우회전 시간만큼 명령 publish
                time.sleep(0.1)
                self.command_pub.publish(control_msg)
                if self.front_distance<50 and cnt==1:
                    break
                
                
            # 0. 1초간 정지
            print('정지')
            control_msg.data = 48 # 정지 명령
            start_time = rospy.get_time()
            while(rospy.get_time()-start_time < 1) : # 1초만큼 정지
                time.sleep(0.1)
                self.command_pub.publish(control_msg)
                
            # 3. 직진
            control_msg.data = 0 # 직진 -> 전방 & 사이드 센서 거리 작을떄(장애물 있을 때) 직진으로 변경
            print("앞 거리: ",self.front_distance," 옆 거리: ", self.side_distance)
            	
            while self.front_distance <50 or self.side_distance<50: #FIXME
            
            	f=self.front_distance
            	s=self.side_distance
            	
            	if f - s > 20 : # FIXME
            	    control_msg.data=-15
            	    
            	elif s - f > 20 :
            	    control_msg.data=17
            	    
            	elif f > 30 and s > 30:
            	    control_msg.data=0
            	    
            	print("앞 거리: ",self.front_distance," 옆 거리: ", self.side_distance)
            	time.sleep(1)
            	self.command_pub.publish(control_msg)
            	
            print('정지')
            control_msg.data = 48 # 정지 명령
            start_time = rospy.get_time()
            while(rospy.get_time()-start_time < 1) : # 1초만큼 정지
                time.sleep(0.1)
                self.command_pub.publish(control_msg)
            	
            print('우회전')
            control_msg.data = -15 #self.second
            print(control_msg.data)
            start_time = rospy.get_time()
            while(rospy.get_time()-start_time < time_right_tilt) : # 우회전 시간만큼 명령 publish
                time.sleep(0.1)
                self.command_pub.publish(control_msg)
               
            print('정지')
            control_msg.data = 48 # 정지 명령
            start_time = rospy.get_time()
            while(rospy.get_time()-start_time < 1) : # 1초만큼 정지
                time.sleep(0.1)
                self.command_pub.publish(control_msg)
                    
            """        
            print('좌회전')
            control_msg.data = 22 #self.first
            start_time = rospy.get_time()
            cnt=0
            while(rospy.get_time()-start_time < time_left_tilt-1) : # 좌회전 시간만큼 명령 publish
                time.sleep(0.1)
                self.command_pub.publish(control_msg)
             
            print('정지')
            control_msg.data = 48 # 정지 명령
            start_time = rospy.get_time()
            while(rospy.get_time()-start_time < 1) : # 1초만큼 정지
                time.sleep(0.1)
                self.command_pub.publish(control_msg)"""
            	
         ########################## 차선인식으로 자율주행 ###########################
        else: # lane change 끝 && 신호등 go
            
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
           self.arduino_command_pub_2() # 아두이노에 최종 명령 publish하는 arduino_command_pub 함수 호출
           rate.sleep()


if __name__ == '__main__':
    
    try:
        Control_Command = ControlCommand() 
        Control_Command.run() # run 함수 호출
        
    except rospy.ROSInterruptException:
        pass 
