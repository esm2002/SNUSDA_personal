#include <Car_Library.h>
#include <SoftwareSerial.h>

int motor_left_1 = 2;
int motor_left_2 = 3;
int motor_right_1 = 5;
int motor_right_2 = 4;
int motor_front_1 = 6;
int motor_front_2 = 7;

int power_L = 100;
int power_R = 102;

int power = 220; //모터속도

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

void setup() {
  pinMode(motor_left_1, OUTPUT);
  pinMode(motor_left_2, OUTPUT);
  pinMode(motor_right_1, OUTPUT);
  pinMode(motor_right_2, OUTPUT);
  pinMode(motor_front_1, OUTPUT);
  pinMode(motor_front_2, OUTPUT);

  Serial.begin(9600);

}

void loop() 
{
  if (Serial.available() > 0) {
    char command = Serial.read();  // 입력된 명령을 읽음

    switch(command)
    {
      case 'f':
        forward();
        break;
        
      case 'b':
        backward();
        break;

      case 'r':
        right_tilt();
        break;
        
      case 'l':
        left_tilt();
        break;
        
      case 's':
        stop_tilt();
        break;
    }
  }
}
