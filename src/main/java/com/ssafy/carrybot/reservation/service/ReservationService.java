package com.ssafy.carrybot.reservation.service;

import com.ssafy.carrybot.reservation.dto.request.VerifyReservationRequestDTO;
import com.ssafy.carrybot.reservation.dto.response.DetailReservationResponseDTO;
import com.ssafy.carrybot.reservation.dto.response.VerifyReservationResponseDTO;

import java.util.List;

public interface ReservationService {
    void startRobotByReservationCode(String reservationCode);
    VerifyReservationResponseDTO verifyReservation(VerifyReservationRequestDTO dto);
    DetailReservationResponseDTO getReservationDetail(Long id);
    DetailReservationResponseDTO getReservationDetailByCode(String reservationCode);
    List<DetailReservationResponseDTO> getAllReservations();

}
