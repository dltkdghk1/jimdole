package com.ssafy.carrybot.robot.controller;

import com.ssafy.carrybot.robot.dto.request.RobotLogSearchCondition;
import com.ssafy.carrybot.robot.dto.response.RobotLogResponseDTO;
import com.ssafy.carrybot.robot.service.RobotLogService;
import lombok.RequiredArgsConstructor;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.*;

import java.util.List;

@RestController
@RequestMapping("/robot/logs")
@RequiredArgsConstructor
public class RobotLogController {

    private final RobotLogService robotLogService;

    @GetMapping
    public ResponseEntity<List<RobotLogResponseDTO>> getAllLogs() {
        return ResponseEntity.ok(robotLogService.getAllLogs());
    }

    @GetMapping(params = "reservationCode")
    public ResponseEntity<List<RobotLogResponseDTO>> getLogsByReservationCode(@RequestParam String reservationCode) {
        return ResponseEntity.ok(robotLogService.getLogsByReservationCode(reservationCode));
    }

    @PostMapping("/search")
    public ResponseEntity<List<RobotLogResponseDTO>> searchLogs(@RequestBody RobotLogSearchCondition condition) {
        return ResponseEntity.ok(robotLogService.searchLogs(condition));
    }
}