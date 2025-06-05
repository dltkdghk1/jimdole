package com.ssafy.carrybot.gate.dto.request;

import lombok.Getter;
import lombok.NoArgsConstructor;
import lombok.Setter;

@Getter
@Setter
@NoArgsConstructor //@RequestBody로 JSON 데이터를 받는 경우에 필요
public class GateRequestDTO {
    private String gateNumber;
    private double x;
    private double y;
}
