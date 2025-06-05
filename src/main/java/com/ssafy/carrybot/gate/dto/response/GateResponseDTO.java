package com.ssafy.carrybot.gate.dto.response;

import lombok.AllArgsConstructor;
import lombok.Getter;

@Getter
@AllArgsConstructor //서비스나 컨트롤러에서 한 번에 생성
public class GateResponseDTO {
    private Long id;
    private String gateNumber;
    private double x;
    private double y;
}

//간단한 DTO → @AllArgsConstructor,
//필드 많고 명확하게 보여줘야 할 때 → @Builder 더 많이 씀