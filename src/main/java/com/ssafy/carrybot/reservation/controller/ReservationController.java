package com.ssafy.carrybot.reservation.controller;

import com.ssafy.carrybot.reservation.dto.request.VerifyReservationRequestDTO;
import com.ssafy.carrybot.reservation.dto.response.DetailReservationResponseDTO;
import com.ssafy.carrybot.reservation.dto.response.VerifyReservationResponseDTO;
import com.ssafy.carrybot.reservation.service.ReservationService;
import lombok.RequiredArgsConstructor;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.*;

import java.util.List;

@RestController
@RequiredArgsConstructor
@RequestMapping("/reservations")
public class ReservationController {

    private final ReservationService reservationService;
    // 예약번호로 로봇 이동 시작
    @PostMapping("/start")
    public ResponseEntity<String> startRobot(@RequestBody VerifyReservationRequestDTO dto) {
        reservationService.startRobotByReservationCode(dto.getReservationCode());
        return ResponseEntity.ok("로봇이 이동을 시작했습니다.");
    }


    @PostMapping("/verify")
    public ResponseEntity<VerifyReservationResponseDTO> verifyReservation(@RequestBody VerifyReservationRequestDTO dto) {
        return ResponseEntity.ok(reservationService.verifyReservation(dto));
    }

    @GetMapping
    public ResponseEntity<List<DetailReservationResponseDTO>> getAllReservations() {
        return ResponseEntity.ok(reservationService.getAllReservations());
    }

    @GetMapping("/{id}")
    public ResponseEntity<DetailReservationResponseDTO> getReservationDetail(@PathVariable Long id) {
        return ResponseEntity.ok(reservationService.getReservationDetail(id));
    }

    @GetMapping("/code/{reservationCode}")
    public ResponseEntity<DetailReservationResponseDTO> getReservationDetailByCode(@PathVariable String reservationCode) {
        return ResponseEntity.ok(reservationService.getReservationDetailByCode(reservationCode));
    }


}