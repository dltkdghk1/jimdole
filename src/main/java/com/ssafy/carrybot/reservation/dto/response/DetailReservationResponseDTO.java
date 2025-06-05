package com.ssafy.carrybot.reservation.dto.response;

import lombok.AllArgsConstructor;
import lombok.Getter;
import lombok.Setter;

import java.time.LocalDateTime;

@Getter
@AllArgsConstructor
public class DetailReservationResponseDTO {
    private Long id;
    private String reservationCode;
    private String phoneNumber;
    private boolean expired;
    private String airline;
    private String destination;
    private LocalDateTime time;
    private String gateNumber;
}
