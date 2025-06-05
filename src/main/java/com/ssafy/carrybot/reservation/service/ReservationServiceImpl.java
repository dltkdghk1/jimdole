package com.ssafy.carrybot.reservation.service;

import com.ssafy.carrybot.flight.entity.Flight;
import com.ssafy.carrybot.gate.entity.Gate;
import com.ssafy.carrybot.reservation.dto.request.VerifyReservationRequestDTO;
import com.ssafy.carrybot.reservation.dto.response.DetailReservationResponseDTO;
import com.ssafy.carrybot.reservation.dto.response.VerifyReservationResponseDTO;
import com.ssafy.carrybot.reservation.entity.Reservation;
import com.ssafy.carrybot.reservation.repository.ReservationRepository;
import com.ssafy.carrybot.robot.entity.Robot;
import com.ssafy.carrybot.robot.repository.RobotRepository;
import com.ssafy.carrybot.robot.service.MqttPublisherService;
import com.ssafy.carrybot.robot.service.RobotLogService;
import jakarta.persistence.EntityNotFoundException;
import lombok.RequiredArgsConstructor;
import lombok.extern.slf4j.Slf4j;
import org.springframework.stereotype.Service;

import java.util.List;
import java.util.stream.Collectors;

@Slf4j
@Service
@RequiredArgsConstructor
public class ReservationServiceImpl implements ReservationService {

    private final ReservationRepository reservationRepository;
    private final MqttPublisherService mqttPublisherService;
    private final RobotLogService robotLogService;
    private final RobotRepository robotRepository;

    @Override
    public void startRobotByReservationCode(String reservationCode) {
        Reservation reservation = reservationRepository
                .findWithFlightAndGateByReservationCodeIgnoreCase(reservationCode.trim())
                .orElseThrow(() -> new EntityNotFoundException("1해당 예약을 찾을 수 없습니다."));

        Flight flight = reservation.getFlight();
        Gate gate = flight.getGate();

        // 로봇 조회 (지금 로봇은 id=1뿐이니깐)
        Robot robot = robotRepository.findById(1L)
                .orElseThrow(() -> new EntityNotFoundException("로봇을 찾을 수 없습니다."));

        // 이동 좌표 계산
        double fromX = robot.getCurrentX();
        double fromY = robot.getCurrentY();
        double toX = gate.getX();
        double toY = gate.getY();


        robotLogService.logMovement(robot, reservationCode, gate.getGateNumber(),
                fromX, fromY, toX, toY); //로그 저장

        mqttPublisherService.sendGateCommand(reservationCode, gate.getGateNumber());
    }

    @Override
    public VerifyReservationResponseDTO verifyReservation(VerifyReservationRequestDTO dto) {
        String code = dto.getReservationCode();
        log.info("검증 요청 받은 예약코드: {}", code);

        Reservation reservation = reservationRepository
                .findWithFlightAndGateByReservationCodeIgnoreCase(code)
                .orElseThrow(() -> {
                    log.error("예약 코드 '{}'를 DB에서 찾지 못했습니다", code);
                    return new EntityNotFoundException("2해당 예약을 찾을 수 없습니다.");
                });

        Flight flight = reservation.getFlight();
        Gate gate = flight.getGate();

        return new VerifyReservationResponseDTO(
                reservation.getId(),
                reservation.getReservationCode(),
                reservation.isExpired(),
                flight.getAirline(),
                flight.getDestination(),
                flight.getTime(),
                gate.getGateNumber()
        );
    }


    @Override
    public DetailReservationResponseDTO getReservationDetail(Long id) {
        Reservation reservation = reservationRepository.findById(id)
                .orElseThrow(() -> new EntityNotFoundException("3예약을 찾을 수 없습니다."));

        Flight flight = reservation.getFlight();
        Gate gate = flight.getGate();

        return new DetailReservationResponseDTO(
                reservation.getId(),
                reservation.getReservationCode(),
                reservation.getPhoneNumber(),
                reservation.isExpired(),
                flight.getAirline(),
                flight.getDestination(),
                flight.getTime(),
                gate.getGateNumber()
        );
    }

    @Override
    public DetailReservationResponseDTO getReservationDetailByCode(String reservationCode) {
        Reservation reservation = reservationRepository
                .findWithFlightAndGateByReservationCodeIgnoreCase(reservationCode)
                .orElseThrow(() -> new EntityNotFoundException("4해당 예약을 찾을 수 없습니다."));

        Flight flight = reservation.getFlight();
        Gate gate = flight.getGate();

        return new DetailReservationResponseDTO(
                reservation.getId(),
                reservation.getReservationCode(),
                reservation.getPhoneNumber(),
                reservation.isExpired(),
                flight.getAirline(),
                flight.getDestination(),
                flight.getTime(),
                gate.getGateNumber()
        );
    }


    @Override
    public List<DetailReservationResponseDTO> getAllReservations() {
        return reservationRepository.findAll().stream()
                .map(reservation -> {
                    Flight flight = reservation.getFlight();
                    Gate gate = flight.getGate();
                    return new DetailReservationResponseDTO(
                            reservation.getId(),
                            reservation.getReservationCode(),
                            reservation.getPhoneNumber(),
                            reservation.isExpired(),
                            flight.getAirline(),
                            flight.getDestination(),
                            flight.getTime(),
                            gate.getGateNumber()
                    );
                })
                .collect(Collectors.toList());
    }



}
