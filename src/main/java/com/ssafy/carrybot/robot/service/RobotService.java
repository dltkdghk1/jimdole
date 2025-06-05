package com.ssafy.carrybot.robot.service;

import com.ssafy.carrybot.robot.dto.response.RobotStatusResponseDTO;

public interface RobotService {
    void startByReservationCode(String reservationCode);
    RobotStatusResponseDTO getRobotStatus(); //예약번호로 로봇 이동 시작
}
