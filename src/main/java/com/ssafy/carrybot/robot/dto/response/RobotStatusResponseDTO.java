package com.ssafy.carrybot.robot.dto.response;

import com.ssafy.carrybot.robot.enums.RobotStatus;
import lombok.AllArgsConstructor;
import lombok.Getter;

import java.time.LocalDateTime;

@Getter
@AllArgsConstructor
public class RobotStatusResponseDTO {
    private double currentX;
    private double currentY;
    private String targetGate;
    private RobotStatus status;
    private LocalDateTime lastUpdated;
}

//서버(Spring) → 로봇 시뮬레이터(MQTT)
//사용자가 직접 로봇에게 데이터 전달하지 않음, 요청 데이터를 받지 않고 MQTT 메시지로 전송하기 때문에 request 필요 없음