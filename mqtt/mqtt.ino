#include <WiFi.h>
#include <PubSubClient.h>  // ✅ TLS 필요 없으니 Secure 삭제

// Wi-Fi 설정
const char* ssid = "";
const char* password = "";

// HiveMQ 퍼블릭 브로커 (비암호화)
const char* mqtt_server = "broker.hivemq.com";
const int mqtt_port = ;

// ✅ 일반 WiFiClient 사용!
WiFiClient espClient;
PubSubClient client(espClient);

void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("✅ WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  IPAddress brokerIP;
  if (WiFi.hostByName(mqtt_server, brokerIP)) {
    Serial.print("✅ broker IP: ");
    Serial.println(brokerIP);
  } else {
    Serial.println("❌ DNS 해석 실패! broker.hivemq.com을 IP로 못 바꿈");
  }
}

void callback(char* topic, byte* payload, unsigned int length) {
  String msg;
  for (unsigned int i = 0; i < length; i++) {
    msg += (char)payload[i];
  }
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("]: ");
  Serial.println(msg);

  int commaIndex = msg.indexOf(',');
  if (commaIndex > 0) {
    float linear = msg.substring(0, commaIndex).toFloat();
    float angular = msg.substring(commaIndex + 1).toFloat();

    Serial.print("Parsed linear: ");
    Serial.print(linear);
    Serial.print(" | angular: ");
    Serial.println(angular);
  }
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");

    if (client.connect("SSADASDASFY51236SADSA")) {
      Serial.println("✅ connected!");
      client.subscribe("turtle/cmd_vel");
    } else {
      Serial.print("❌ failed, rc=");
      Serial.print(client.state());
      Serial.println(" → retrying in 5 seconds...");
      delay(5000);
    }
  }
}

void setup() {
  Serial.begin(115200);
  setup_wifi();

  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
}
