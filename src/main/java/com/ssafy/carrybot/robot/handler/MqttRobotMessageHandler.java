package com.ssafy.carrybot.robot.handler;

import com.fasterxml.jackson.databind.ObjectMapper;
import com.ssafy.carrybot.photo.service.MqttPhotoReceiverService;
import com.ssafy.carrybot.robot.dto.mqtt.RobotStatusMessageDTO;
import com.ssafy.carrybot.robot.entity.Robot;
import com.ssafy.carrybot.robot.entity.RobotLog;
import com.ssafy.carrybot.robot.enums.EventType;
import com.ssafy.carrybot.robot.enums.RobotStatus;
import com.ssafy.carrybot.robot.repository.RobotLogRepository;
import com.ssafy.carrybot.robot.repository.RobotRepository;
import com.ssafy.carrybot.robot.service.MqttPublisherService;
import com.ssafy.carrybot.robot.websocket.RobotWebSocketService;
import jakarta.persistence.EntityNotFoundException;
import lombok.RequiredArgsConstructor;
import org.springframework.stereotype.Component;

import java.time.LocalDateTime;

@Component
@RequiredArgsConstructor
public class MqttRobotMessageHandler { //로봇이 게이트로 이동해서 도착 후 사진찍음

    private final RobotRepository robotRepository;
    private final RobotLogRepository robotLogRepository;
    private final MqttPublisherService mqttPublisherService;
    private final RobotWebSocketService robotWebSocketService; //웹소켓
    private final MqttPhotoReceiverService mqttPhotoReceiverService;

    public void handleStatus(String payload) {

        System.out.println("📩 handleStatus() 호출됨! 수신된 payload: " + payload);

        try {
            ObjectMapper mapper = new ObjectMapper();
            RobotStatusMessageDTO dto = mapper.readValue(payload, RobotStatusMessageDTO.class);

            if ("STOPPED".equals(dto.getEvent_type())) {
                handleStopped(dto.getRobot_id(), dto.getX(), dto.getY(), dto.getReservation_code());
            }


        } catch (Exception e) {
            System.err.println("❌ MQTT 상태 처리 중 오류: " + e.getMessage());
            e.printStackTrace();
        }
    }



    private void handleStopped(int robotId, double x, double y, String reservationCode) {
        Robot robot = robotRepository.findById((long) robotId)
                .orElseThrow(() -> new EntityNotFoundException("로봇을 찾을 수 없습니다."));

        // 1. 상태 갱신
        robot.setStatus(RobotStatus.STOPPED);
        robot.setCurrentX(x); // 시뮬레이터가 보내는 MQTT 메시지에서 도착 좌표 가져옴
        robot.setCurrentY(y);
        robot.setLastUpdated(LocalDateTime.now());

        robotRepository.save(robot);

        // 2. 로그 저장
        RobotLog log = RobotLog.builder()
                .robot(robot)
                .eventType(EventType.STOPPED)
                .event("로봇이 목적지에 도착함")
                .timestamp(LocalDateTime.now())
                .reservationCode(reservationCode)
                .gateNumber(robot.getTargetGate())
                .toX(robot.getCurrentX())
                .toY(robot.getCurrentY())
                .build();
        robotLogRepository.save(log);

        // 3. 사진 요청
        //mqttPublisherService.sendPhotoRequest(robotId);

        // 📢 웹소켓 알림 전송
        robotWebSocketService.sendRobotStatus(
                robot.getCurrentX(),
                robot.getCurrentY(),
                robot.getTargetGate(),
                robot.getStatus(),
                robot.getLastUpdated(),
                "로봇이 게이트 " + robot.getTargetGate() + "에 도착했습니다.",
                reservationCode
        );
    }


    public void handleLog(String payload) {
        System.out.println("📘 [LOG] " + payload);
        // 필요 시: 추후 로봇 로그 테이블에 저장 가능
    }

    public void handlePhoto(String payload) {
        System.out.println("📷 [PHOTO] handlePhoto() 호출됨!");
        System.out.println("📷 수신된 payload: " + payload);
        // 필요 시: 이미지 URL 저장 또는 DB 연동


        mqttPhotoReceiverService.handlePhoto(payload);
    }
}
