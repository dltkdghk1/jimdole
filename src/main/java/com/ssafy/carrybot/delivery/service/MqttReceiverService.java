package com.ssafy.carrybot.delivery.service;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import org.eclipse.paho.client.mqttv3.MqttClient;
import org.eclipse.paho.client.mqttv3.MqttConnectOptions;
import org.eclipse.paho.client.mqttv3.MqttException;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.beans.factory.annotation.Value;
import org.springframework.stereotype.Service;

@Service
public class MqttReceiverService {
    private final DeliveryServiceImpl deliveryService;


    @Value("${MQTT_BROKER}")
    private String broker;

    @Value("${MQTT_CLIENT_ID}")
    private String clientId;

    @Value("${MQTT_USERNAME}")
    private String username;

    @Value("${MQTT_PASSWORD}")
    private String password;

    @Autowired
    public MqttReceiverService(DeliveryServiceImpl deliveryService) {
        this.deliveryService = deliveryService;
    }

    public void startMqttClient() throws MqttException {
        MqttClient client = new MqttClient(broker, clientId);
        MqttConnectOptions options = new MqttConnectOptions();
        options.setUserName(username);
        options.setPassword(password.toCharArray());

        client.connect(options);

        // robot/status 토픽 구독
        client.subscribe("robot/status", (topic, msg) -> handleMqttMessage(topic, msg));
    }

    // MQTT 메시지 처리
    private void handleMqttMessage(String topic, org.eclipse.paho.client.mqttv3.MqttMessage msg) {
        String payload = new String(msg.getPayload());

        if ("robot/status".equals(topic)) {
            // 상태 메시지 처리
            String eventType = extractEventType(payload);  // event_type 추출
            Long deliveryId = extractDeliveryId(payload);  // deliveryId 추출
            deliveryService.updateDeliveryStatus(deliveryId, eventType);  // Delivery 상태 업데이트
        }

    }

    // 이벤트타입 추출
    private String extractEventType(String payload) {
        try {
            // Jackson ObjectMapper 생성
            ObjectMapper objectMapper = new ObjectMapper();

            // payload (JSON 문자열)을 JsonNode로 파싱
            JsonNode rootNode = objectMapper.readTree(payload);

            // eventType을 JsonNode에서 추출
            String eventType = rootNode.path("event_type").asText(); // "event_type" 필드의 값을 추출

            return eventType;
        } catch (Exception e) {
            // 예외 처리
            throw new IllegalArgumentException("Payload에서 eventType 추출 실패: " + e.getMessage());
        }
    }

    // deliveryId 추출
    private Long extractDeliveryId(String payload) {
        try {
            // Jackson ObjectMapper 생성
            ObjectMapper objectMapper = new ObjectMapper();

            // payload (JSON 문자열)을 JsonNode로 파싱
            JsonNode rootNode = objectMapper.readTree(payload);

            // 'delivery_id' 대신 'robot_id'를 추출
            Long robotId = rootNode.path("robot_id").asLong(); // 'robot_id' 필드의 값을 추출

            return robotId;
        } catch (Exception e) {
            // 예외 처리
            throw new IllegalArgumentException("Payload에서 deliveryId 추출 실패: " + e.getMessage());
        }
    }
}
