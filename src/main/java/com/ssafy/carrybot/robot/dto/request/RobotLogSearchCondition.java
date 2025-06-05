package com.ssafy.carrybot.robot.dto.request;

import com.ssafy.carrybot.robot.enums.EventType;
import lombok.Getter;
import lombok.Setter;

import java.time.LocalDate;

@Setter
@Getter
public class RobotLogSearchCondition {
    private String reservationCode;
    private EventType eventType;
    private LocalDate startDate;
    private LocalDate endDate;
}
//로봇 로그 관련 요청 DTO