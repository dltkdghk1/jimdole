package com.ssafy.carrybot.delivery.dto.response;

import lombok.AllArgsConstructor;
import lombok.Getter;

@Getter
@AllArgsConstructor
public class DeliveryResponseDTO {

    private Long id;
    private String reservationCode;
    private String status;

}
