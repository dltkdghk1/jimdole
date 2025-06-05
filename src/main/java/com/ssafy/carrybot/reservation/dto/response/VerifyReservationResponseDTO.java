package com.ssafy.carrybot.reservation.dto.response;

import lombok.AllArgsConstructor;
import lombok.Getter;

import java.time.LocalDateTime;

@Getter
@AllArgsConstructor
public class VerifyReservationResponseDTO {

    private Long id;
    private String reservationCode;
    private boolean expired;
    private String airline;
    private String destination;
    private LocalDateTime time;
    private String gateNumber;
}

//예약 번호의 유효 여부를 클라이언트에게 알려주는 응답 DTO