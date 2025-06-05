#include <WiFi.h>
#include <PubSubClient.h>
#include "config.h"
#include <ArduinoJson.h>
#include <Arduino.h>


WiFiClient wifiClient;
PubSubClient client(wifiClient);

int count = 0;

float linear = 0.0;
float angular = 0.0;

String reservation_code = "";
String event_type = "STOPPED";
int robot_id = 0;
float x = 0.0;
float y = 0.0;
String gate = "";


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

// wifi 설정 함수
void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(WIFI_SSID);

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD); // wifi 연결
  
  // wifi 연결될 때까지 대기
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("✅ WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  IPAddress brokerIP;
  if (WiFi.hostByName(MQTT_BROKER, brokerIP)) {
    Serial.print("✅ broker IP: ");
    Serial.println(brokerIP);
  } else {
    Serial.println("❌ DNS 해석 실패! broker주소를 IP로 못 바꿈");
  }
}

// 시물레이션의 바퀴 각속도를 output값에 맞게 바꾸기
int mapSpeedToPWM(float wheel_speed) {
  int pwm = (int)((wheel_speed + 1.18) / 0.104);
  return pwm;
}

// 오른쪽 모터 제어
void RcontrolMotor(int r_wheel) {

  int speed = abs(r_wheel) + 40;
  speed = min(speed, 255);

  if (r_wheel > 0) {
    digitalWrite(pinRF, HIGH);
    digitalWrite(pinRB, LOW);
    analogWrite(MotorRFPWM, speed);
  } else if (r_wheel < 0) {
    digitalWrite(pinRF, LOW);
    digitalWrite(pinRB, HIGH);
    analogWrite(MotorRBPWM, speed);
  } else {
    digitalWrite(pinRF, LOW);
    digitalWrite(pinRB, LOW);
  }
}

// 왼쪽 모터 제어
void LcontrolMotor(int l_wheel) {

  int speed = abs(l_wheel) + 10;
  speed = min(speed, 255);

  if (l_wheel > 0) {
    digitalWrite(pinLF, HIGH);
    digitalWrite(pinLB, LOW);
    analogWrite(MotorLFPWM, speed);
  } else if (l_wheel < 0) {
    digitalWrite(pinLF, LOW);
    digitalWrite(pinLB, HIGH);
    analogWrite(MotorLBPWM, speed);
  } else {
    digitalWrite(pinLF, LOW);
    digitalWrite(pinLB, LOW);
  }
}

void sendMessage() {
  // JSON 형식 문자열 만들기
  String payload = "{";
  payload += "\"reservation_code\":\"" + reservation_code + "\",";
  payload += "\"event_type\":\"" + event_type + "\",";
  payload += "\"robot_id\":" + String(robot_id) + ",";
  payload += "\"x\":" + String(x, 3) + ",";
  payload += "\"y\":" + String(y, 3) + ",";
  payload += "\"gate\":\"" + gate + "\"";
  payload += "}";

  const char* topic = "robot/status";

  if (client.connected()) {
    client.publish(topic, payload.c_str());
    Serial.println("✅ MQTT 메시지 전송 완료: " + payload);
  } else {
    Serial.println("❌ MQTT 서버에 연결되어 있지 않음!");
  }
}

// 통신이 오면 콜백함수 실행 됨
void callback(char* topic, byte* payload, unsigned int length) {
  String payloadStr;

  for (unsigned int i = 0; i < length; i++) {
    payloadStr += (char)payload[i];
  }

  payloadStr.trim();

  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("]: ");
  Serial.println(payloadStr);

  if (strcmp(topic, "robot/gate")==0) {
    StaticJsonDocument<256> doc;
    DeserializationError error = deserializeJson(doc, payload);

    if (error) {
      Serial.print("JSON 파싱 실패: ");
      Serial.println(error.c_str());
      return;
    }
    
    String reservationCodeStr = doc["reservation_code"];
    reservation_code = reservationCodeStr;

    String robotidStr = doc["robot_id"];
    robot_id = robotidStr.toInt();

    String xStr = doc["x"];
    x = xStr.toFloat();

    String yStr = doc["y"];
    y = yStr.toFloat();

    String gateStr = doc["gate"];
    gate = gateStr;

    Serial.print("reservation_code : ");
    Serial.println(reservation_code);
    Serial.print("event_type : ");
    Serial.println(event_type);
    Serial.print("robot_id");
    Serial.println(robot_id);
    Serial.print("x : ");
    Serial.println(x);
    Serial.print("y : ");
    Serial.println(y);
    Serial.print("gate : ");
    Serial.println(gate);
  } else {
    int commaIndex = payloadStr.indexOf(',');
    if (commaIndex > 0) {
      String linearStr = payloadStr.substring(0, commaIndex);
      String angularStr = payloadStr.substring(commaIndex + 1);

      linear = abs(linearStr.toFloat());
      angular = angularStr.toFloat();
    }

    Serial.print("Parsed linear: ");
    Serial.print(linear);
    Serial.print(" | angular: ");
    Serial.println(angular);

    // 바퀴 속도 계산
    float r_wheel_speed = (2 * linear - angular * sim_separation) / (2 * sim_radius);
    Serial.print("오른쪽 : ");
    Serial.println(r_wheel_speed);

    float l_wheel_speed = (2 * linear + angular * sim_separation) / (2 * sim_radius);
    Serial.print("왼쪽 : ");
    Serial.println(l_wheel_speed);



    int r_wheel = mapSpeedToPWM(r_wheel_speed);
    int l_wheel = mapSpeedToPWM(l_wheel_speed);

    Serial.print("오른쪽 값 : ");
    Serial.println(r_wheel);

    Serial.print("왼쪽 값 : ");
    Serial.println(l_wheel);

    if (r_wheel == 11 && l_wheel == 11) {
      count++;
      Serial.println(count);
    }

    RcontrolMotor(r_wheel);
    LcontrolMotor(l_wheel);
    
    if (count >= 20) {
      sendMessage();
      count = 0;
     }
  }
}

void reconnect() {

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD); // wifi 연결
  
  // wifi 연결될 때까지 대기
  while (WiFi.status() != WL_CONNECTED) {
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("✅ WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  IPAddress brokerIP;
  if (WiFi.hostByName(MQTT_BROKER, brokerIP)) {
    Serial.print("✅ broker IP: ");
    Serial.println(brokerIP);
  } else {
    Serial.println("❌ DNS 해석 실패! broker.hivemq.com을 IP로 못 바꿈");
  }

  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");

    String clientId = "arduinoClient-" + String(random(0xffff), HEX);

    if (client.connect(MQTT_CLIENT_ID, MQTT_USERNAME, MQTT_PASSWORD)) {
      Serial.println("✅ connected!");
      client.subscribe("turtle/imu");
      client.subscribe("robot/gate");
    } else {
      WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
      Serial.print("❌ failed, rc=");
      Serial.print(client.state());
      Serial.println(" → retrying in 5 seconds...");
      delay(5000);
    }
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

  setup_wifi();
  client.setServer(MQTT_BROKER, MQTT_PORT);
  client.setCallback(callback);
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
}
