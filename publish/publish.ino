#include <WiFi.h>
#include <PubSubClient.h>
#include "config.h"  // 여기에 WIFI_SSID, WIFI_PASSWORD, MQTT_BROKER, MQTT_CLIENT_ID, MQTT_USERNAME, MQTT_PASSWORD, MQTT_PORT 정의되어 있어야 함

WiFiClient wifiClient;
PubSubClient client(wifiClient);

// WiFi 연결 함수
void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(WIFI_SSID);

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\n✅ WiFi connected");
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

// MQTT 재연결 함수
void reconnect() {
  Serial.println("\n⏳ MQTT 서버 재연결 시도 중...");
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    String clientId = "arduinoClient-" + String(random(0xffff), HEX);

    if (client.connect(MQTT_CLIENT_ID, MQTT_USERNAME, MQTT_PASSWORD)) {
      Serial.println("✅ MQTT connected!");
    } else {
      Serial.print("❌ failed, rc=");
      Serial.print(client.state());
      Serial.println(" → 5초 후 재시도");
      delay(5000);
    }
  }
}

// 메시지 publish 함수
void sendMessage() {
  String payload = "{\"reservation_code\":\"LOG027\",\"event_type\":\"STOPPED\",\"robot_id\":1,\"x\":-61.974,\"y\":-53.253,\"gate\":\"A\"}";
  const char* topic = "robot/status";

  if (client.connected()) {
    client.publish(topic, payload.c_str());
    Serial.println("✅ MQTT 메시지 전송 완료: " + payload);
  } else {
    Serial.println("❌ MQTT 서버에 연결되어 있지 않음!");
  }
}

// MQTT 콜백 함수 (필요하면 내용 추가)
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("📩 메시지 수신 [");
  Serial.print(topic);
  Serial.print("]: ");

  for (unsigned int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}

void setup() {
  Serial.begin(115200);

  setup_wifi();
  client.setServer(MQTT_BROKER, MQTT_PORT);
  client.setCallback(callback);

  reconnect();        // MQTT 연결
  sendMessage();      // ✅ 딱 한 번만 메시지 전송
  delay(1000);
}

// 반복적으로 MQTT 메시지를 publish
void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
}
