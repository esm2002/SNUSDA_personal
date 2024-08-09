#include <ros.h>

#include <Car_Library.h>
#include <ros.h>
#include <std_msgs/Int16.h>

ros::NodeHandle nh;
std_msgs::Int16 message;


int motor_left_1 = 2;
int motor_left_2 = 3;

int motor_right_1 = 5;
int motor_right_2 = 4;

int motor_front_1 = 7;
int motor_front_2 = 6;

int analogPin = A4;

int power_L = 230;
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

void motor(const std_msgs::Int16& command){
// int angle = 0;
  forward();
  if(command.data < 25){ // left
// angle = 17;
    change_angle(command.data);
    
  }
  
//  if(command.data == 2){ // front
//    angle = 1;
//    change_angle(angle);
//    forward();
//  }
  
//  if(command.data == 3){ // right
//    angle = -13;
//    change_angle(angle);
//    forward();
//  }

 //if(command.data == 4){
 // int angle = -10;
 // change_angle(angle);
 // forward();
 // } 
  
  if(command.data == 44){ 
  while(1){
    pause();
  }
  }
 
  else{
  forward();
  motor_hold(motor_front_1, motor_front_2);
  }
 
}
  
ros::Subscriber<std_msgs::Int16> sub("final_val", motor);

void setup() {

  Serial.begin(57600);

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
  
  val_mid = ((val_max + val_min)/2-1);
  
  nh.initNode();
  nh.subscribe(sub);
  
  
}

void loop() {
  while(once){
    left_tilt();
    if (potentiometer_Read(analogPin) >= val_mid){
      once = 0;
      stop_tilt();
      break;
    }
  } 
  nh.spinOnce();

  // forward();
}
