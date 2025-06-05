package com.ssafy.carrybot.robot.websocket;

import com.ssafy.carrybot.robot.enums.RobotStatus;
import lombok.RequiredArgsConstructor;
import org.springframework.messaging.simp.SimpMessagingTemplate;
import org.springframework.stereotype.Service;

import java.time.LocalDateTime;

@Service
@RequiredArgsConstructor
public class RobotWebSocketService {

    private final SimpMessagingTemplate messagingTemplate;

    public void sendRobotStatus(double currentX, double currentY, String targetGate, RobotStatus status, LocalDateTime lastUpdated, String message, String reservationCode) {
        RobotWebSocketMessageDTO dto = new RobotWebSocketMessageDTO(
                message,
                reservationCode,
                targetGate,
                currentX,
                currentY,
                status,
                lastUpdated
        );

        messagingTemplate.convertAndSend("/topic/robot/status", dto);
    }
}
