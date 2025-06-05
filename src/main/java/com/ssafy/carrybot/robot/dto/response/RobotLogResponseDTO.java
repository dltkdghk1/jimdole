package com.ssafy.carrybot.robot.dto.response;

import com.ssafy.carrybot.robot.enums.EventType;
import lombok.AllArgsConstructor;
import lombok.Getter;

import java.time.LocalDateTime;

@Getter
@AllArgsConstructor
public class RobotLogResponseDTO {
    private Long id;
    private EventType eventType;
    private String event;
    private LocalDateTime timestamp;
    private String reservationCode;
    private String gateNumber;
    private double fromX;
    private double fromY;
    private double toX;
    private double toY;
}