#include <Servo.h> 

int pinLB=4;     // 왼쪽 바퀴 뒤로 회전
int pinLF=5;     // 왼쪽 바퀴 앞으로 회전
int pinRF=6;    // 오른쪽 바퀴 앞으로 회전
int pinRB=7;    // 오른쪽 바퀴 뒤로 회전

int MotorLFPWM=5;  // 왼쪽 바퀴 앞으로 회전 속도 조절 
int MotorRFPWM=6;  // 오른쪽 바퀴 앞으로 회전 속도 조절
int MotorLBPWM=4; // 왼쪽 바퀴 뒤로 회전 속도 조절
int MotorRBPWM=7; // 오른쪽바퀴 뒤로 회전 속도 조절

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);     // Initialize 
      
  pinMode(pinLB,OUTPUT); // Define 4 pin for the output (PWM)
  pinMode(pinLF,OUTPUT); // Define 5 pin for the output (PWM)
  pinMode(pinRB,OUTPUT); // Define 6 pin for the output (PWM) 
  pinMode(pinRF,OUTPUT); // Define 7 pin for the output (PWM)
  
  pinMode(MotorLFPWM,OUTPUT);  // Define 5 pin for PWM output 
  pinMode(MotorRFPWM,OUTPUT);  // Define 6 pin for PWM output
  pinMode(MotorLBPWM,OUTPUT);  // Define 4 pin for PWM output
  pinMode(MotorRBPWM,OUTPUT);  // Define 7 pin for PWM output
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(pinRF,HIGH);  // 오른쪽 바퀴 앞으로 high면 활성화 low면 비활성화
  digitalWrite(pinRB,LOW);   // 오른쪽 바퀴 뒤로 high면 활성화 low면 비활성화
  analogWrite(MotorRFPWM,60);// 바퀴 속도 조절

  digitalWrite(pinLF,HIGH);  // 왼쪽 바퀴 앞으로 high면 활성화 low면 비활성화
  digitalWrite(pinLB,LOW);   // 왼쪽 바퀴 뒤로 high면 활성화 low면 비활성화
  analogWrite(MotorLFPWM,60);//Set the output speed(PWM)
}
