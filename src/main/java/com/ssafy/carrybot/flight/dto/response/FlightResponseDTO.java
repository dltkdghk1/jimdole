package com.ssafy.carrybot.flight.dto.response;

import lombok.AllArgsConstructor;
import lombok.Getter;

import java.time.LocalDateTime;

@Getter
@AllArgsConstructor
public class FlightResponseDTO {

    private Long id;
    private String airline;
    private String destination;
    private LocalDateTime time;
    private Long gateId;
    private String gateNumber;
}

//서버 -> 클라이언트(조회 시)
//등록된 flight의 전체 정보