#include <Servo.h> 

int pinLB=4;     // 왼쪽 바퀴 뒤로 회전
int pinLF=5;     // 왼쪽 바퀴 앞으로 회전
int pinRF=6;    // 오른쪽 바퀴 앞으로 회전
int pinRB=7;    // 오른쪽 바퀴 뒤로 회전

int MotorLFPWM=5;  // 왼쪽 바퀴 앞으로 회전 속도 조절 
int MotorRFPWM=6;  // 오른쪽 바퀴 앞으로 회전 속도 조절
int MotorLBPWM=4; // 왼쪽 바퀴 뒤로 회전 속도 조절
int MotorRBPWM=7; // 오른쪽바퀴 뒤로 회전 속도 조절

// 시물레이터 터틀봇
const float sim_radius = 3.3/100.0; // 바퀴 반지름 m
const float sim_separation = 10.9/100.0; // 바퀴 사이 거리 m

float linear = 0.3;
float angular = 0.1;

// 시물레이션 상의 속도를 정량적 속도로 바꾸기
// speed_sim은 
int mapSpeedToPWM(float wheel_speed) {
  int pwm = (int)((wheel_speed + 1.18) / 0.104 );

  // PWM 값이 0~255 범위를 넘지 않도록 제한
  pwm = constrain(pwm, 0, 255);
  return pwm;
}

// 오른쪽 모터 제어
void RcontrolMotor(int r_wheel) {
  if (r_wheel > 0) {
    digitalWrite(pinRF, HIGH);
    digitalWrite(pinRB, LOW);
    analogWrite(MotorRFPWM, abs(r_wheel));
  } else if (r_wheel < 0) {
    digitalWrite(pinRF, LOW);
    digitalWrite(pinRB, HIGH);
    analogWrite(MotorRBPWM, abs(r_wheel));
  } else {
    digitalWrite(pinRF, LOW);
    digitalWrite(pinRB, LOW);
  }
}

// 왼쪽 모터 제어
void LcontrolMotor(int l_wheel) {
  if (l_wheel > 0) {
    digitalWrite(pinLF, HIGH);
    digitalWrite(pinLB, LOW);
    analogWrite(MotorLFPWM, abs(l_wheel));
  } else if (l_wheel < 0) {
    digitalWrite(pinLF, LOW);
    digitalWrite(pinLB, HIGH);
    analogWrite(MotorLBPWM, abs(l_wheel));
  } else {
    digitalWrite(pinLF, LOW);
    digitalWrite(pinLB, LOW);
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);     // Initialize 
      
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
  Serial.print("선속도 : ");
  Serial.println(linear);

  Serial.print("각속도 : ");
  Serial.println(angular);
  // put your main code here, to run repeatedly:
  // digitalWrite(pinRF,HIGH);  // 오른쪽 바퀴 앞으로 high면 활성화 low면 비활성화
  // digitalWrite(pinRB,LOW);   // 오른쪽 바퀴 뒤로 high면 활성화 low면 비활성화
  // analogWrite(MotorRFPWM,100);// 바퀴 속도 조절

  // digitalWrite(pinLF,HIGH);  // 왼쪽 바퀴 앞으로 high면 활성화 low면 비활성화
  // digitalWrite(pinLB,LOW);   // 왼쪽 바퀴 뒤로 high면 활성화 low면 비활성화
  // analogWrite(MotorLFPWM,100);//Set the output speed(PWM)

  // 바퀴 속도 계산
  float r_wheel_speed = (2 * linear - angular * sim_separation) / (2 * sim_radius);
  Serial.print("오른쪽 : ");
  Serial.println(r_wheel_speed);

  float l_wheel_speed = (2 * linear + angular * sim_separation) / (2 * sim_radius);
  Serial.print("왼쪽 : ");
  Serial.println(l_wheel_speed);

  // 바퀴 속도를 정량적인 값으로 변경
  int r_wheel = mapSpeedToPWM(r_wheel_speed);
  Serial.print("오른쪽 값 : ");
  Serial.println(r_wheel);
  int l_wheel = mapSpeedToPWM(l_wheel_speed);
  Serial.print("왼쪽 값 : ");
  Serial.println(l_wheel);

  // 방향 고려한 모터 제어
  RcontrolMotor(r_wheel);
  LcontrolMotor(l_wheel);
}
