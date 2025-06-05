package com.ssafy.carrybot.robot.service;

import com.ssafy.carrybot.flight.entity.Flight;
import com.ssafy.carrybot.gate.entity.Gate;
import com.ssafy.carrybot.reservation.entity.Reservation;
import com.ssafy.carrybot.reservation.repository.ReservationRepository;
import com.ssafy.carrybot.robot.dto.response.RobotStatusResponseDTO;
import com.ssafy.carrybot.robot.entity.Robot;
import com.ssafy.carrybot.robot.entity.RobotLog;
import com.ssafy.carrybot.robot.enums.EventType;
import com.ssafy.carrybot.robot.enums.RobotStatus;
import com.ssafy.carrybot.robot.repository.RobotLogRepository;
import com.ssafy.carrybot.robot.repository.RobotRepository;
import jakarta.persistence.EntityNotFoundException;
import lombok.RequiredArgsConstructor;
import org.springframework.stereotype.Service;

import java.time.LocalDateTime;

@Service
@RequiredArgsConstructor
public class RobotServiceImpl implements RobotService {

    private final ReservationRepository reservationRepository;
    private final RobotRepository robotRepository;
    private final RobotLogRepository robotLogRepository;
    private final MqttPublisherService mqttPublisherService;
    private final RobotLogService robotLogService; //이동중 로그 추가

    @Override
    public void startByReservationCode(String reservationCode) {
        // 예약 조회 + Flight + Gate까지 fetch
        Reservation reservation = reservationRepository
                .findWithFlightAndGateByReservationCodeIgnoreCase(reservationCode)
                .orElseThrow(() -> new EntityNotFoundException("예약이 존재하지 않습니다."));

        Flight flight = reservation.getFlight();
        Gate gate = flight.getGate();

        Robot robot = robotRepository.findById(1L)
                .orElseThrow(() -> new EntityNotFoundException("로봇이 존재하지 않습니다."));

        double fromX = robot.getCurrentX();
        double fromY = robot.getCurrentY();
        // 로봇 상태 갱신
        robot.setTargetGate(gate.getGateNumber());
        robot.updateStatus(RobotStatus.MOVING);
        robotRepository.save(robot);

        // 로그 저장
//        robotLogRepository.save(RobotLog.builder()
//                .robot(robot)
//                .eventType(EventType.MOVED)
//                .event("게이트 " + gate.getGateNumber() + "으로 이동 시작")
//                .timestamp(LocalDateTime.now())
//                .reservationCode(reservationCode)
//                .gateNumber(gate.getGateNumber())
//                .fromX(robot.getCurrentX())
//                .fromY(robot.getCurrentY())
//                .toX(gate.getX())
//                .toY(gate.getY())
//                .build());
        robotLogService.logMovement(robot, reservationCode, gate.getGateNumber(),
                fromX, fromY, gate.getX(), gate.getY());

        // MQTT 전송
        mqttPublisherService.sendGateCommand(reservationCode, gate.getGateNumber());
    }

    @Override
    public RobotStatusResponseDTO getRobotStatus() {
        Robot robot = robotRepository.findById(1L)
                .orElseThrow(() -> new EntityNotFoundException("로봇이 존재하지 않습니다."));

        return new RobotStatusResponseDTO(
                robot.getCurrentX(),
                robot.getCurrentY(),
                robot.getTargetGate(),
                robot.getStatus(),
                robot.getLastUpdated()
        );
    }
}

