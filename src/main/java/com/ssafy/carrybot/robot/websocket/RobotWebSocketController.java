package com.ssafy.carrybot.robot.websocket;

import com.ssafy.carrybot.reservation.dto.request.VerifyReservationRequestDTO;
import com.ssafy.carrybot.reservation.dto.response.VerifyReservationResponseDTO;
import com.ssafy.carrybot.reservation.service.ReservationService;
import com.ssafy.carrybot.robot.service.MqttPublisherService;
import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.springframework.messaging.handler.annotation.MessageMapping;
import org.springframework.messaging.simp.SimpMessagingTemplate;
import org.springframework.stereotype.Controller;

@Slf4j
@Controller
@RequiredArgsConstructor
public class RobotWebSocketController {

    private final ReservationService reservationService;
    private final MqttPublisherService mqttPublisherService;
    private final SimpMessagingTemplate messagingTemplate;

    @MessageMapping("/verifyReservation")
    public void handleReservationVerification(VerifyReservationRequestDTO dto) {
        log.info("웹소켓으로 받은 예약코드: {}", dto.getReservationCode());

        // 1. 예약 검증
        VerifyReservationResponseDTO response = reservationService.verifyReservation(dto);

        // 2. MQTT 로봇 명령 전송
        mqttPublisherService.sendGateCommand(dto.getReservationCode(), response.getGateNumber());

        // 3. 프론트에 상태 전송
        messagingTemplate.convertAndSend("/topic/robot/status", response);
    }
}