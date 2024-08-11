#include <Car_Library.h>
#include <ros.h>
#include <std_msgs/Int16.h>

ros::NodeHandle nh;
std_msgs::Int16 command_msg;
std_msgs::Int16 angle_msg;

int motor_left_1 = 2;
int motor_left_2 = 3;
int motor_right_1 = 5;
int motor_right_2 = 4;
int motor_front_1 = 7;
int motor_front_2 = 6;
int analogPin = A4; // Potentiometer pin

int power_L = 230;
int power_R = 255;
int power = 150;

int val_max;
int val_min;
int val_mid;

ros::Publisher angle_pub("steering_angle", &angle_msg);

void change_angle(int angle){
  //change_angle(5); 정방향
  while((potentiometer_Read(analogPin) - val_mid) >= (angle)){
    right_tilt();
    if ((potentiometer_Read(analogPin) - val_mid) <= (angle +1) && (potentiometer_Read(analogPin) - val_mid) >= (angle - 1)){
      stop_tilt();
      break;
    }
  }

  while((potentiometer_Read(analogPin) - val_mid) <= (angle)){
    left_tilt();
    if ((potentiometer_Read(analogPin) - val_mid) >= (angle -1) && (potentiometer_Read(analogPin) - val_mid) <= (angle + 1)){
      stop_tilt();
      break;
    }
  }
}

// 전진 함수
void forward() {
  motor_forward(motor_right_1, motor_right_2, power_R);
  motor_forward(motor_left_1, motor_left_2, power_L);
}

void left_tilt(){
  motor_forward(motor_front_1, motor_front_2, power);
}

void right_tilt(){
  motor_backward(motor_front_1, motor_front_2, power);
}
// 좌회전 및 전진 함수
void left_tilt_forward() {
  motor_forward(motor_front_1, motor_front_2, power);
  forward();
}

// 우회전 및 전진 함수
void right_tilt_forward() {
  motor_backward(motor_front_1, motor_front_2, power);
  forward();
}

// 중앙 조향 및 전진 함수
void center_tilt_forward() {
  stop_tilt();
  forward();
}

// 조향 멈춤 함수
void stop_tilt() {
  motor_hold(motor_front_1, motor_front_2);
}

// 모든 모터 멈춤 함수
void pause() {
  motor_hold(motor_right_1, motor_right_2);
  motor_hold(motor_left_1, motor_left_2);
  stop_tilt();
}

// 초기 조향 설정 함수
void initialize_steering() {
  int once = 1;
  while (once) {
    left_tilt();
    if (potentiometer_Read(analogPin) >= val_mid) {
      once = 0;
      stop_tilt();
      break;
    }
  } 
}

// 명령어 콜백 함수
void commandCallback(const std_msgs::Int16& command_msg) {
  int command = command_msg.data;
  if (command == 1) { // Center tilt + Forward
    change_angle(0);
    delay(50);
    forward();
  } else if (command == 2) { // Left tilt + Forward
    left_tilt_forward();
  } else if (command == 3) { // Right tilt + Forward
    right_tilt_forward();
  } else if (command == 4) { // Stop
    pause();
  }
}

// ROS 구독자 설정
ros::Subscriber<std_msgs::Int16> sub("steering_command", commandCallback);

void setup() {
  Serial.begin(57600);

  pinMode(motor_left_1, OUTPUT);
  pinMode(motor_left_2, OUTPUT);
  pinMode(motor_right_1, OUTPUT);
  pinMode(motor_right_2, OUTPUT);
  pinMode(motor_front_1, OUTPUT);
  pinMode(motor_front_2, OUTPUT);
  pinMode(analogPin, INPUT);

  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(angle_pub);

  // 포텐셔미터 최대값, 최소값, 중간값 설정
  left_tilt();
  delay(4000);
  stop_tilt();
  val_max = potentiometer_Read(analogPin);
  Serial.println(val_max);
  
  right_tilt();
  delay(4000);
  stop_tilt();
  val_min = potentiometer_Read(analogPin);
  Serial.println(val_min);
  
  val_mid = (val_max + val_min) / 2;
  Serial.println(val_mid);
  initialize_steering();
}

void loop() {
  nh.spinOnce();
  
  int analog_value = potentiometer_Read(analogPin); // Read analog value from potentiometer
  angle_msg.data = analog_value;
  angle_pub.publish(&angle_msg);

  delay(100); // Publish at 10 Hz
}
