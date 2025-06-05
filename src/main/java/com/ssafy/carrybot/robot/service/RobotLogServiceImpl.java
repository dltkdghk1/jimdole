package com.ssafy.carrybot.robot.service;

import com.ssafy.carrybot.robot.dto.request.RobotLogSearchCondition;
import com.ssafy.carrybot.robot.dto.response.RobotLogResponseDTO;
import com.ssafy.carrybot.robot.entity.Robot;
import com.ssafy.carrybot.robot.entity.RobotLog;
import com.ssafy.carrybot.robot.enums.EventType;
import com.ssafy.carrybot.robot.repository.RobotLogRepository;
import com.ssafy.carrybot.robot.spec.RobotLogSpecification;
import lombok.RequiredArgsConstructor;
import org.springframework.stereotype.Service;

import java.math.BigDecimal;
import java.time.LocalDateTime;
import java.time.ZoneId;
import java.util.List;
import java.util.stream.Collectors;

@Service
@RequiredArgsConstructor
public class RobotLogServiceImpl implements RobotLogService {

    private final RobotLogRepository robotLogRepository;

    @Override
    public List<RobotLogResponseDTO> getAllLogs() {
        return robotLogRepository.findAllByOrderByTimestampDesc().stream()
                .map(this::toDTO)
                .collect(Collectors.toList());
    }

    @Override
    public List<RobotLogResponseDTO> getLogsByReservationCode(String code) {
        return robotLogRepository.findByReservationCode(code).stream()
                .map(this::toDTO)
                .collect(Collectors.toList());
    }

    @Override
    public List<RobotLogResponseDTO> searchLogs(RobotLogSearchCondition condition) {
        return robotLogRepository.findAll(
                RobotLogSpecification.search(
                        condition.getReservationCode(),
                        condition.getEventType(),
                        condition.getStartDate(),
                        condition.getEndDate()

                )
        ).stream().map(this::toDTO).collect(Collectors.toList());
    }

    @Override
    public void logMovement(Robot robot, String reservationCode, String gateNumber,
                            double fromX, double fromY, double toX, double toY) {

        // ✅ 한국 시간으로 명시적 시간 생성
        LocalDateTime now = LocalDateTime.now(ZoneId.of("Asia/Seoul"));

        RobotLog log = RobotLog.builder()
                .robot(robot)
                .eventType(EventType.MOVED)
                .event("게이트 " + gateNumber + "으로 이동 시작")
                .timestamp(now)
                .reservationCode(reservationCode)
                .gateNumber(gateNumber)
                .fromX(fromX)
                .fromY(fromY)
                .toX(toX)
                .toY(toY)
                .build();

        robotLogRepository.save(log);
    }



    private RobotLogResponseDTO toDTO(RobotLog log) {
        return new RobotLogResponseDTO(
                log.getId(),
                log.getEventType(),
                log.getEvent(),
                log.getTimestamp(),
                log.getReservationCode(),
                log.getGateNumber(),
                log.getFromX(),
                log.getFromY(),
                log.getToX(),
                log.getToY()
        );
    }
}