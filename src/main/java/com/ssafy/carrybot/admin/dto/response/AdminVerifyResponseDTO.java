package com.ssafy.carrybot.admin.dto.response;

import lombok.AllArgsConstructor;
import lombok.Getter;

@Getter
@AllArgsConstructor
public class AdminVerifyResponseDTO {
    private String code; // SU or FA
    private String message; // 성공 or 실패 메시지
}