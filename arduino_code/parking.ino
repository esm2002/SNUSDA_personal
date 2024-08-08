#include <Car_Library.h>
#include <ros.h>
#include <std_msgs/Int16.h>

ros::NodeHandle nh;
std_msgs::Int16 message;


int motor_left_1 = 2; //전진
int motor_left_2 = 3; //후진

int motor_right_1 = 5; //전진
int motor_right_2 = 4; //후진

int motor_front_1 = 7; //좌로 틀기
int motor_front_2 = 6; //우로 틀기

int analogPin = A4;

int power_L = 65;
int power_R = 60;
//int power_L = 122;
//int power_R = 125;
// left power 170
// right power 230
int val;
int val_max;
int val_min;
int val_mid;

int power = 150;

//초음파 센서
int TR_Rfront = 8; //오른쪽 앞
int EC_Rfront = 9;
long Rfront_dist;
float Rfront_ctrl;

int TR_Rside = 10; //오른쪽 옆
int EC_Rside = 11;
long Rside_dist;
float Rside_ctrl;

int TR_Rback = 12; //오른쪽 뒤
int EC_Rback = 13;
long Rback_dist;
float Rback_ctrl;

int TR_Lfront = 14; //왼쪽 앞
int EC_Lfront = 15;
long Lfront_dist;
float Lfront_ctrl;

int TR_Lside = 16; //왼쪽 옆
int EC_Lside = 17;
long Lside_dist;
float Lside_ctrl;

int TR_Lback = 18; //왼쪽 뒤
int EC_Lback = 19;
long Lback_dist;
float Lback_ctrl;

//PID 파라미터 설정(FIXME)
float Kp = 1.0;
float Ki = 0.1;
float Kd = 0.05;
float target_dist = 50.0; // 목표 거리 (단위: cm)
float prev_error = 0.0;
float integral = 0.0;

int once = 1;
int parking_step = 0; // not total_parking mode


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

float computePID(float target, float current) {
  float error = target - current;
  integral += error;
  float derivative = error - prev_error;
  float output = Kp * error + Ki * integral + Kd * derivative;
  prev_error = error;
  return output;
}



long getDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH);
  long distance = duration * 0.034 / 2; // duration/29/2;
  return distance;
}

int count1 = 0;
int count11 = 0;
int count2 = 0;
int count22 = 0;
int first_car = 0;
int second_car = 0;

int back_angle = 0;
int print_Lf = 0;
int print_Ls = 0;
int print_Lb = 0;
int print_Rf = 0;
int print_Rs = 0;
int print_Rb = 0;

int try_Rback = 0;

void motor(const std_msgs::Int16& command){
  
  switch (parking_step) {
    case 0:
    
    forward();
    motor_hold(motor_front_1, motor_front_2); //앞바퀴 조향 고정
    Rback_dist = getDistance(TR_Rback, EC_Rback);
    Rfront_dist = getDistance(TR_Rfront, EC_Rfront);

    if(command.data){
      count1 += 1;
    }
    if(count1 > 5){
      count1 = 0;
      first_car = 1; 
    }
    if(first_car && (Rback_dist < 200)){
      count11 += 1;
    }
    if((count11 > 10) && (Rfront_dist < 200)){
      parking_step = 1;
    }
    break;
  
    case 1: //앞으로 좌회전
    Rside_dist = getDistance(TR_Rside, EC_Rside);
    delay(10);
    Rback_dist = getDistance(TR_Rback, EC_Rback);
    change_angle(20);
    forward();
    motor_hold(motor_front_1, motor_front_2);

    if(Rback_dist > 200){
      count2 += 1;
    }
    if(count2 > 5){
      count2 = 0;
      second_car = 1; 
    }
    if(second_car && (Rside_dist > 200)){
      count22 += 1;
    }
    if(count22 > 5){
      back_angle = -18;
      parking_step = 2;
    }
    break;

    case 2: //뒤로 우회전
    Lback_dist = getDistance(TR_Lback, EC_Lback);
    delay(5);
    Lside_dist = getDistance(TR_Lside, EC_Lside);
    delay(5);
    Rback_dist = getDistance(TR_Rback, EC_Rback);
    delay(5);
    Rside_dist = getDistance(TR_Rside, EC_Rside);

    change_angle(back_angle);
    backward();
    motor_hold(motor_front_1, motor_front_2);

    if(((Rback_dist < 70) && (Lback_dist < 120)) || (try_Rback == 1)){
      try_Rback = 1;
      power_L = 50; //80
      power_R = 45; //75
      
      back_angle += 2;
      back_angle = constrain(back_angle, -20, 10);
      if((Lside_dist < 90) && (Rside_dist < 90)){ //(abs(abs(Lside_dist-Rside_dist)-abs(Lback_dist-Rback_dist)) < 30)
        pause();
        delay(1000);
        back_angle = 0;
        parking_step = 3;
      }
      else{
        
      }
    }

    break;
  
    case 3: //후진 주차
    power_L = 50; //80
    power_R = 45; //75
    Lback_dist = getDistance(TR_Lback, EC_Lback);
    delay(5);
    Rback_dist = getDistance(TR_Rback, EC_Rback);
    delay(5);
    // Lside_dist = getDistance(TR_Lside, EC_Lside);
    // delay(50);
    // Rside_dist = getDistance(TR_Rside, EC_Rside);
    // delay(50);
    Lfront_dist = getDistance(TR_Lfront, EC_Lfront);
    delay(5);
    Rfront_dist = getDistance(TR_Rfront, EC_Rfront);

    change_angle(back_angle);
    backward();
    motor_hold(motor_front_1, motor_front_2);

    if((Rfront_dist < 100) && (Lfront_dist < 100) && (Rback_dist > 300) && (Lback_dist > 300)){
      pause();
      delay(4000);
      parking_step = 4;
    }

    // if(Rback_dist < 25){
    //   change_angle(back_angle-20);
    //   forward();
    //   motor_hold(motor_front_1, motor_front_2);
    //   delay(1000);
    // }

    break;
  
    case 4: //leaving mode
    power_L = 80;
    power_R = 75;
    Lside_dist = getDistance(TR_Lside, EC_Lside);
    delay(10);
    Rside_dist = getDistance(TR_Rside, EC_Rside);
    if((Rside_dist < 80) && (Lside_dist < 80)){ //주차공간에서 빠져나가는 중
      change_angle(0);
      forward();
      motor_hold(motor_front_1, motor_front_2);
    }
    else{ //빠져나왔을 때
      change_angle(-20);
      forward();
      motor_hold(motor_front_1, motor_front_2);
      delay(10000);
      parking_step = 5;
    }
    break;

    case 5:
    change_angle(0);
    forward();
    motor_hold(motor_front_1, motor_front_2);

    break;
  }
}
  
ros::Subscriber<std_msgs::Int16> sub("parking_val", motor);


void setup() {

  Serial.begin(57600);
  pinMode(motor_left_1,OUTPUT);
  pinMode(motor_left_2,OUTPUT);

  pinMode(motor_right_1,OUTPUT);
  pinMode(motor_right_2,OUTPUT);
  
  pinMode(motor_front_1,OUTPUT);
  pinMode(motor_front_2,OUTPUT);
  pinMode(analogPin,INPUT);

  delay(1000);
  left_tilt();
  Serial.println("Left Tilted in Initial");
  delay(2000);
  stop_tilt();
  Serial.println("Stop Tilted in Initial");
  val_max= potentiometer_Read(analogPin);
  Serial.print("Val Max : ");
  Serial.println(val_max);

  
  right_tilt();
  Serial.println("Right Tilted in Initial");
  delay(2000);
  stop_tilt();
  Serial.println("Stop Tilted in Initial");
  val_min= potentiometer_Read(analogPin);
  Serial.print("Val Min : ");
  Serial.println(val_min);
  val_mid = ((val_max + val_min)/2)-1;
  Serial.print("Val Mid : ");
  Serial.println(val_mid);

  nh.initNode();
  nh.subscribe(sub);
  
  // 초음파 센서 핀 모드 설정 (오른쪽)
  pinMode(TR_Rfront, OUTPUT);
  pinMode(EC_Rfront, INPUT);
  pinMode(TR_Rside, OUTPUT);
  pinMode(EC_Rside, INPUT);
  pinMode(TR_Rback, OUTPUT);
  pinMode(EC_Rback, INPUT);

  // 초음파 센서 핀 모드 설정 (왼쪽)
  pinMode(TR_Lfront, OUTPUT);
  pinMode(EC_Lfront, INPUT);
  pinMode(TR_Lside, OUTPUT);
  pinMode(EC_Lside, INPUT);
  pinMode(TR_Lback, OUTPUT);
  pinMode(EC_Lback, INPUT);
  
}

void loop() {

  while(once){
    left_tilt();
    Serial.println("Left Tilted in Main Loop");
    Serial.print("Potentiometer_Read : ");
    Serial.println(potentiometer_Read(analogPin));
    
    if (potentiometer_Read(analogPin) > val_mid){
      Serial.println("Stop Tilted in Main Loop");
      once = 0;
      stop_tilt();
      break;
    }

  } 

  nh.spinOnce();
  delay(1);

}
