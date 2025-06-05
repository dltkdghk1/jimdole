package com.ssafy.carrybot.reservation.dto.request;

import lombok.Getter;
import lombok.NoArgsConstructor;
import lombok.Setter;

@Getter
@Setter
@NoArgsConstructor
public class VerifyReservationRequestDTO {

    private String reservationCode;
}


//예약 번호를 입력 받을 때 사용하는 요청 DTO
//@Getter
//@Setter
//@NoArgsConstructor  이 3개는 리퀘스트 적을 때 무조건 적자!