#include <Car_Library.h>
#include <ros.h>
#include <std_msgs/Int16.h>
#include <SoftwareSerial.h>

SoftwareSerial BTSerial(19, 18); //RX=19, TX=18

ros::NodeHandle nh;
std_msgs::Int16 message;


int motor_left_1 = 2;
int motor_left_2 = 3;

int motor_right_1 = 5;
int motor_right_2 = 4;

int motor_front_1 = 6;
int motor_front_2 = 7;

int analogPin = A0;

int power_L = 240;
int power_R = 255;
// left power 170
// right power 230
int val;
int val_max;
int val_min;
int val_mid;


int power = 150;

int TR_front = 11;
int EC_front = 10;

int TR_right = 13;
int EC_right = 12;
int once = 1;

void forward(){
  motor_forward(motor_right_1, motor_right_2, power_R);
  motor_forward(motor_left_1, motor_left_2, power_L);
}
void backward(){
  motor_backward(motor_right_1, motor_right_2, power_R);
  motor_backward(motor_left_1, motor_left_2, power_L);
}
void left_tilt(){
  motor_forward(motor_front_1, motor_front_2, power);
}

void right_tilt(){
  motor_backward(motor_front_1, motor_front_2, power);
}
void stop_tilt(){
  motor_hold(motor_front_1, motor_front_2);
}
void pause(){
  motor_hold(motor_right_1,motor_right_2);
  motor_hold(motor_left_1, motor_left_2);
}

void change_angle(int angle){
  while((potentiometer_Read(analogPin) - val_mid) >= (angle)){
    right_tilt();
    if ((potentiometer_Read(analogPin) - val_mid) <= (angle +1) && (potentiometer_Read(analogPin) - val_mid) >= (angle) - 1){
      stop_tilt();
      break;
    }
  }

  while((potentiometer_Read(analogPin) - val_mid) <= (angle)){
    left_tilt();
    if ((potentiometer_Read(analogPin) - val_mid) >= (angle -1) && (potentiometer_Read(analogPin) - val_mid) <= (angle) + 1){
      stop_tilt();
      break;
    }
  }
}

void setup() {

  Serial.begin(9600);
  BTSerial.begin(9600);

  pinMode(motor_left_1,OUTPUT);
  pinMode(motor_left_2,OUTPUT);

  pinMode(motor_right_1,OUTPUT);
  pinMode(motor_right_2,OUTPUT);
  
  pinMode(motor_front_1,OUTPUT);
  pinMode(motor_front_2,OUTPUT);
  pinMode(analogPin,INPUT);
  
  nh.initNode();
  
  
}

void loop() 
{
  if (BTSerial.available()) // 블루투스로부터 데이터를 받았는지 확인
  {
    char cmd = BTSerial.read(); //받은 데이터 읽기

    switch(cmd)
    {
      case 'F':
        forward();
        break;
      case 'B':
        backward();
        break;
      case 'L':
        left_tilt();
        break;
      case 'R':
        right_tilt();
        break;
      case 'S':
        stop_tilt();
        break;
    }
  } 
  nh.spinOnce();

}
