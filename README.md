# S1_rev1
서울대학교 미래모빌리티기술센터 여름/겨울 현장실습용 SW 코드

## setting & requirements

- CPU: i7 155H(up to 4.8GHz 24MB L3 Cache) / VGA: NVIDIA GeForce RTX 4050 / RAM: 32GB
- Linux Ubuntu 20.04.6 / NVIDIA Driver == 535.54.03
- ROS noetic

**For end - to - steer(시간측정 경기)**
- CUDA == 12.2
- cuDNN == 8.9.7
- python == 3.8.10
- tensorflow == 2.13.1
- torch == 2.2.1

**For traffic-light-detect(미션수행 경기)**
- ultralytics(YOLO) == 8.1.34

**For Obstacle-detection(미션수행 경기)**
- sklearn == 1.3.2

## 1. workspace set up & clone repository

```bash
$ mkdir ~/catkin_ws
$ cd ~/catkin_ws
$ git clone https://github.com/FMTC-S1-EDU/S1_rev1.git
```

## 2. Install required packages

**For camera** [참고](https://wiki.ros.org/usb_cam)
```bash
$ sudo apt install ros-noetic-usb-cam
$ cd ~/catkin_ws/src
$ git clone https://github.com/ros-drivers/rosserial.git
```
**For lidar** [참고](https://github.com/Slamtec/rplidar_ros)
```bash
$ cd ~/catkin_ws/src
$ git clone https://github.com/Slamtec/rplidar_ros.git
```
**For arduino** [참고](https://wiki.ros.org/rosserial)
```bash
$ sudo apt-get install ros-noetic-rosserial-arduino
$ sudo apt-get install ros-noetic-rosserial

$ cd ~/catkin_ws/src
$ git clone https://github.com/ros-drivers/rosserial.git
```

## 3. Build the Workspace

```bash
$ cd ~/catkin_ws
$ catkin_make
$ catkin_make install
```
  
## 4. Run 

### 1. 시간측정경기

1. upload arduino code
: lane_tracer_for_race.ino

2. end-to-steer
```bash
$ cd ~/catkin_ws
$ source devel/setup.bash
$ roslaunch race total.launch # run rosserial and camera
$ rosrun race DecisionMaker_for_race.py
```
---------------------------------------------
### 2. 미션수행경기

1. upload arduino code
: run.ino

2. lane-masking
```bash
$ cd ~/catkin_ws
$ source devel/setup.bash
$ roslaunch usb_cam multi_usb_cam.launch # run two camera(lane-detection and traffic-light-detection)
$ rosrun test lane_masking_re.py
```

3. obstacle-detection
```bash
$ roslaunch lidar lidar_final.launch
```

4. traffic-light-detection
```bash
$ rosrun camera yolo_final.py
$ rosrun camera traffic_light_color_decoder_final.py
```

5. final decision
```bash
$ rosrun control test_traffic_decision.py
```

6. run rosserial (connect with arduino)
check the port name : _port:=/dev/ttyACM0 _baud:=57600
```bash
$ rosrun rosserial_python serial_node.py
```

---------------------------------------------
### 3. 미션수행경기_주차 미션

1. upload arduino code
: parking.ino

2. parking-detection
```bash
$ cd ~/catkin_ws
$ source devel/setup.bash
$ roslaunch lidar lidar_parking.launch
```

3. run rosserial (connect with arduino)
check the port name : _port:=/dev/ttyACM0 _baud:=57600
```bash
$ rosrun rosserial_python serial_node.py
```