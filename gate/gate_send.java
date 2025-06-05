import org.eclipse.paho.client.mqttv3.*;
import org.json.JSONObject;

public class GatePublisher {

    public static void main(String[] args) {
        // ✅ 브로커 정보 (너가 사용하는 정보로 수정!)
        String broker = "tcp://your-mqtt-broker.com:1883";  // 예: "tcp://j12e104.p.ssafy.io:1883"
        String clientId = "javaGatePublisher";
        String username = "robot";        // ← 사용자 계정
        String password = "robot104";     // ← 사용자 비번
        String topic = "robot/gate"; // 토픽은 ros2랑 이름 맞춘거기 때문에 수정 절대 금지 env파일에 넣어서 쓰는것도 금지지

        try {
            // ✅ MQTT 연결 설정
            MqttClient client = new MqttClient(broker, clientId);
            MqttConnectOptions options = new MqttConnectOptions();
            options.setUserName(username);
            options.setPassword(password.toCharArray());
            client.connect(options);

            // ✅ 전송할 JSON 데이터
            JSONObject payload = new JSONObject();
            payload.put("reservation_id", "A1234"); // 여기는 디비에서 가져오시면 됩니다 !
            payload.put("gate", "G15");

            // ✅ MQTT 메시지 생성 및 퍼블리시
            MqttMessage message = new MqttMessage(payload.toString().getBytes());
            message.setQos(1); // QoS 1: 최소 1번 전달
            client.publish(topic, message);

            System.out.println("✅ 게이트 정보 전송 완료!");
            System.out.println("Payload: " + payload.toString());

            client.disconnect();
            System.out.println("✅ MQTT 연결 종료");

        } catch (MqttException e) {
            System.out.println("❌ 오류 발생: " + e.getMessage());
            e.printStackTrace();
        }
    }
}
