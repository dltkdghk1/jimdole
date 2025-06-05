package com.ssafy.carrybot.sms.dto.request;

import lombok.Getter;
import lombok.Setter;

@Getter
@Setter
public class SmsRequestDTO {
    private String messageContent; // 메시지 내용
    private Long reservationId;     // 예약 아이디 (reservationId 추가)
}
