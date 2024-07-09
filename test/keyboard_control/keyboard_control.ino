#include <Car_Library.h>
#include "ros.h"
#include <std_msgs/Int16.h>

ros::NodeHandle nh;
std_msgs::Int16 message;
ros::Publisher pub("Pot_value", &message);

int motor_left_1 = 2;
int motor_left_2 = 3;

int motor_right_1 = 5;
int motor_right_2 = 4;

int motor_front_1 = 6;
int motor_front_2 = 7;

int analogPin = A0;

int power_L = 50;
int power_R = 430;
// left power 75
// right power 100
int val;
int val_max;
int val_min;
int val_mid;

int power = 100;

int TR_front = 11;
int EC_front = 10;

int TR_right = 13;
int EC_right = 12;

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
void motor(const std_msgs::Int16& command){

  if(command.data == 1){ // left
  //0~255
  left_tilt();
  delay(100);
  motor_hold(motor_front_1, motor_front_2);
  forward();
  }
  
  if(command.data == 2){ // front
  motor_hold(motor_front_1, motor_front_2);
  forward();
  }
  
  if(command.data == 3){ // right
  right_tilt();
  delay(100);
  motor_hold(motor_front_1, motor_front_2);
  forward();
  }

  if(command.data == 4){
  pause();
  while(1){ }
  }
  
  else{
  forward();
  motor_hold(motor_front_1, motor_front_2);
  }
}
  
ros::Subscriber<std_msgs::Int16> sub("keyboard_input", motor);

void setup() {
  init();
  pinMode(motor_left_1,OUTPUT);
  pinMode(motor_left_2,OUTPUT);

  pinMode(motor_right_1,OUTPUT);
  pinMode(motor_right_2,OUTPUT);
  
  pinMode(motor_front_1,OUTPUT);
  pinMode(motor_front_2,OUTPUT);
  pinMode(analogPin,INPUT);
  
  left_tilt();
  delay(4000);
  stop_tilt();
  val_max= potentiometer_Read(analogPin);
  
  right_tilt();
  delay(4000);
  stop_tilt();
  val_min= potentiometer_Read(analogPin);
  
  val_mid = (val_max + val_min)/2;
  
  nh.initNode();
  nh.advertise(pub);
  nh.subscribe(sub);
  
  int input = 0;
  
  
  if(val < val_mid){
    left_tilt();
    val = potentiometer_Read(analogPin);
    if(val == val_mid){
      stop_tilt();
    }
  }
}

void loop() {

  val = potentiometer_Read(analogPin);
  
  nh.spinOnce();
  message.data = potentiometer_Read(analogPin) - val_mid;
  pub.publish(&message);
}
