package com.ssafy.carrybot.delivery.dto.request;

import lombok.Getter;
import lombok.NoArgsConstructor;

@Getter
@NoArgsConstructor
public class DeliveryRequestDTO {
    private String reservationCode;  // 예약번호
    private String status;
}
