package com.ssafy.carrybot.sms.dto.response;

import lombok.AllArgsConstructor;
import lombok.Getter;

@Getter
@AllArgsConstructor
public class SmsResponseDTO {
    private String status;  // 전송 상태 (예: SUCCESS, FAILED)
    private String message; // 전송 상태 메시지
}
