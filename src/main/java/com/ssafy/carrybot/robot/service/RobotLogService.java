package com.ssafy.carrybot.robot.service;

import com.ssafy.carrybot.robot.dto.request.RobotLogSearchCondition;
import com.ssafy.carrybot.robot.dto.response.RobotLogResponseDTO;
import com.ssafy.carrybot.robot.entity.Robot;

import java.math.BigDecimal;
import java.util.List;

public interface RobotLogService {
    List<RobotLogResponseDTO> getAllLogs();
    List<RobotLogResponseDTO> getLogsByReservationCode(String reservationCode);
    List<RobotLogResponseDTO> searchLogs(RobotLogSearchCondition condition);
    void logMovement(Robot robot, String reservationCode, String gateNumber,
                     double fromX, double fromY, double toX, double toY); //로봇 이동중 -> 로그에 기록

}
