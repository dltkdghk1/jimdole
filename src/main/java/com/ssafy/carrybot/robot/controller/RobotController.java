package com.ssafy.carrybot.robot.controller;

import com.ssafy.carrybot.robot.dto.response.RobotStatusResponseDTO;
import com.ssafy.carrybot.robot.service.RobotService;
import lombok.RequiredArgsConstructor;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.*;

@RestController
@RequiredArgsConstructor
@RequestMapping("/robot")
public class RobotController {

    private final RobotService robotService;

    // 예약번호로 로봇 출발
    @PostMapping("/start/{reservationCode}")
    public ResponseEntity<Void> startByReservationCode(@PathVariable String reservationCode) {
        robotService.startByReservationCode(reservationCode);
        return ResponseEntity.ok().build();
    }

    // 현재 로봇 상태 조회
    @GetMapping("/status")
    public ResponseEntity<RobotStatusResponseDTO> getRobotStatus() {
        return ResponseEntity.ok(robotService.getRobotStatus());
    }
}
