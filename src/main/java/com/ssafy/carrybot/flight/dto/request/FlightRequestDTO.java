package com.ssafy.carrybot.flight.dto.request;

import lombok.Getter;
import lombok.NoArgsConstructor;
import lombok.Setter;

import java.time.LocalDateTime;

@Getter
@Setter
@NoArgsConstructor
public class FlightRequestDTO {
    private String airline;
    private String destination;
    private LocalDateTime time;
    private Long gateId; // 외부에서는 Gate의 id만 보내주면 됨
}
//클라이언트 -> 서버(등록 시)