package com.ssafy.carrybot.robot.service;

import com.fasterxml.jackson.databind.ObjectMapper;
import com.ssafy.carrybot.gate.entity.Gate;
import com.ssafy.carrybot.gate.repository.GateRepository;
import lombok.RequiredArgsConstructor;
import org.eclipse.paho.client.mqttv3.MqttClient;
import org.eclipse.paho.client.mqttv3.MqttMessage;
import org.springframework.beans.factory.annotation.Qualifier;
import org.springframework.stereotype.Service;

import java.util.HashMap;
import java.util.Map;

@Service
@RequiredArgsConstructor
public class MqttPublisherService {

    @Qualifier("mqttPublisherClient")
    private final MqttClient mqttClient;
    private final GateRepository gateRepository;
    private final ObjectMapper objectMapper = new ObjectMapper();
    private final String topic = "robot/gate";

    public void sendGateCommand(String reservationCode, String gateNumber) {
        try {
            // MQTT 연결 확인 및 재연결 대기
            if (!mqttClient.isConnected()) {
                mqttClient.reconnect();

                int retries = 0;
                while (!mqttClient.isConnected() && retries++ < 5) {
                    System.out.println("🔄 MQTT 재연결 시도 중... (" + retries + ")");
                    Thread.sleep(500); // 0.5초 대기
                }

                if (!mqttClient.isConnected()) {
                    throw new IllegalStateException("❌ MQTT 연결 실패: 브로커에 연결되지 않음");
                }
            }

            Gate gate = gateRepository.findByGateNumber(gateNumber)
                    .orElseThrow(() -> new IllegalArgumentException("❌ 게이트 좌표 정보 없음: " + gateNumber));

            Map<String, Object> payloadMap = new HashMap<>();
            payloadMap.put("robot_id", 1);
            payloadMap.put("reservation_code", reservationCode);
            payloadMap.put("gate", gateNumber);
            payloadMap.put("x", gate.getX()); //int에서 double로 바꿈
            payloadMap.put("y", gate.getY());
            payloadMap.put("event_type", "MOVED"); //백엔드가 시뮬레이터에게 보내는 MQTT 메시지 내용,실제 전송되는 JSON 정보임

            String payload = objectMapper.writeValueAsString(payloadMap);

            MqttMessage message = new MqttMessage(payload.getBytes());
            message.setQos(1);
            mqttClient.publish(topic, message);

            System.out.println("✅ 게이트 명령 전송 성공: " + payload);

        } catch (Exception e) {
            System.err.println("❌ MQTT 전송 에러: " + e.getMessage());
            e.printStackTrace();
        }
    } //gate정보 백에서 시뮬레이터에 넘기기

//    public void sendPhotoRequest(int robotId) {
//        try {
//            if (!mqttClient.isConnected()) {
//                mqttClient.reconnect();
//            }
//
//            Map<String, Object> payloadMap = new HashMap<>();
//            payloadMap.put("robot_id", robotId);
//            payloadMap.put("event_type", "PHOTO_SEND");
//
//            String payload = objectMapper.writeValueAsString(payloadMap);
//            MqttMessage message = new MqttMessage(payload.getBytes());
//            message.setQos(1);
//            mqttClient.publish("robot/status", message);
//
//            System.out.println("📸 사진 요청 전송 완료: " + payload);
//        } catch (Exception e) {
//            System.err.println("❌ 사진 요청 MQTT 전송 실패: " + e.getMessage());
//        }
//    } //사진 요청 메소드

}
