package com.ssafy.carrybot.robot.websocket;

import com.ssafy.carrybot.robot.enums.RobotStatus;
import lombok.AllArgsConstructor;
import lombok.Getter;

import java.time.LocalDateTime;

@Getter
@AllArgsConstructor
public class RobotWebSocketMessageDTO {
    private String message;          // ex. 로봇이 게이트 A에 도착했습니다.
    private String reservationCode;
    private String gateNumber;

    private double currentX;
    private double currentY;
    private RobotStatus status;
    private LocalDateTime lastUpdated; //웹소켓으로 로봇 상태 전체를 프론트에 전송하는 데 필요한 정보들 추가.
}
//이제 웹소켓으로 로봇 상태 전체 + 도착 메시지를 프론트에 보낼 수 있음