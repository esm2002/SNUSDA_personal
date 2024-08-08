#include <Car_Library.h>
#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>

ros::NodeHandle nh;
std_msgs::Int16 message;

std_msgs::Int32 Distance; // 앞*10000 + 뒤 값으로 표시
ros::Publisher dist("distance",&Distance);


int motor_left_1 = 2;
int motor_left_2 = 3;

int motor_right_1 = 5;
int motor_right_2 = 4;

int motor_front_1 = 7;
int motor_front_2 = 6;

int analogPin = A4;

int power_L = 80;
int power_R = 75;
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

int once=1;

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
  
  if(command.data < 20){ // left
// angle = 17;
    change_angle(command.data);
    forward();
    
  }
  
  if(command.data == 48){ 
    pause();
  }


  else{
    forward();
    motor_hold(motor_front_1, motor_front_2);
  }
 
}
  
ros::Subscriber<std_msgs::Int16> sub("control_command_pub", motor);

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

void setup() {

  Serial.begin(57600);
  pinMode(motor_left_1,OUTPUT);
  pinMode(motor_left_2,OUTPUT);

  pinMode(motor_right_1,OUTPUT);
  pinMode(motor_right_2,OUTPUT);
  
  pinMode(motor_front_1,OUTPUT);
  pinMode(motor_front_2,OUTPUT);
  pinMode(analogPin,INPUT);
  
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

  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(dist);
  
  
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
  Distance.data = getDistance(TR_Rfront,EC_Rfront)*10000+getDistance(TR_Rside,EC_Rside);
  dist.publish(&Distance);
  nh.spinOnce();
  delay(1);

}
