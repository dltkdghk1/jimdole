import org.eclipse.paho.client.mqttv3.*;
import java.util.Base64;
import java.io.FileOutputStream;
import java.io.IOException;
import org.json.JSONObject;

public class MqttPhotoReceiver {

    public static void main(String[] args) {
        String broker = "tcp://your-broker-ip:1883"; // 예: "tcp://broker.hivemq.com:1883"
        String clientId = "JavaPhotoReceiver";
        String username = "robot";
        String password = "robot104";
        String topic = "robot/photo"; // 토픽은 ros2랑 이름 맞춘거기 때문에 수정 절대 금지 env파일에 넣어서 쓰는것도 금지지

        try {
            MqttClient client = new MqttClient(broker, clientId);
            MqttConnectOptions options = new MqttConnectOptions();
            options.setUserName(username);
            options.setPassword(password.toCharArray());

            client.connect(options);
            System.out.println("✅ MQTT 연결됨");

            client.subscribe(topic, (t, msg) -> {
                try {
                    String payload = new String(msg.getPayload());
                    JSONObject json = new JSONObject(payload);

                    String reservationId = json.getString("reservation_id");
                    String imageBase64 = json.getString("image_base64");

                    System.out.println("✅ 사진 수신 완료");
                    System.out.println("예약번호: " + reservationId);
                    System.out.println("base64 길이: " + imageBase64.length());

                    // "data:image/jpeg;base64,..." 제거
                    if (imageBase64.contains(",")) {
                        imageBase64 = imageBase64.split(",")[1];
                    }

                    // 디코딩 및 파일 저장
                    byte[] imageBytes = Base64.getDecoder().decode(imageBase64);
                    String filename = "received_photo_" + reservationId + ".jpg";

                    try (FileOutputStream fos = new FileOutputStream(filename)) {
                        fos.write(imageBytes);
                        System.out.println("✅ 이미지 저장 완료: " + filename);
                    }

                } catch (Exception e) {
                    System.out.println("❌ 메시지 처리 오류: " + e.getMessage());
                }
            });

            System.out.println("📡 사진 수신 대기 중...");
        } catch (MqttException e) {
            e.printStackTrace();
        }
    }
}
