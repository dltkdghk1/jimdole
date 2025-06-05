package com.ssafy.carrybot.photo.dto.request;

import lombok.Getter;
import lombok.Setter;

@Getter
@Setter
public class PhotoRequestDTO {

    // 업로드할 이미지의 Base64 인코딩된 이미지 데이터
    private String imageBase64;

    // ReservationId 추가
    private Long reservationId;  // 예약 아이디
}
