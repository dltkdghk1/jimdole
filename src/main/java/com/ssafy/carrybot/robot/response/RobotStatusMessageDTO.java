package com.ssafy.carrybot.robot.response;

import lombok.AllArgsConstructor;
import lombok.Getter;

@Getter
@AllArgsConstructor
public class RobotStatusMessageDTO {
    private String message;        // ex. "로봇이 게이트 A에 도착했습니다."
    private String reservationCode;
    private String gateNumber;
}