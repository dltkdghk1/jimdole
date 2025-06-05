package com.ssafy.carrybot.photo.controller;

import com.ssafy.carrybot.photo.dto.request.PhotoRequestDTO;
import com.ssafy.carrybot.photo.dto.response.PhotoResponseDTO;
import com.ssafy.carrybot.photo.service.PhotoService;
import com.ssafy.carrybot.sms.dto.request.SmsRequestDTO;
import com.ssafy.carrybot.sms.service.SmsService;
import org.springframework.http.HttpStatus;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.*;

@RestController
@RequestMapping("/photos")
public class PhotoController {

    private final PhotoService photoService;
    private final SmsService smsService;  // SmsService를 주입받기 위한 필드 추가


    // 생성자 주입
    public PhotoController(PhotoService photoService, SmsService smsService) {
        this.photoService = photoService;
        this.smsService = smsService;
    }

    // 사진 업로드 API
    @PostMapping("/upload")
    public ResponseEntity<PhotoResponseDTO> uploadPhoto(@RequestBody PhotoRequestDTO photoRequestDTO) {
        // photoRequestDTO에서 Base64 이미지 데이터를 가져와 서비스에 전달
        Long reservationId = photoRequestDTO.getReservationId();
        PhotoResponseDTO response = photoService.savePhotoToS3AndDb(photoRequestDTO.getImageBase64(), reservationId);

        // 예약 ID를 사용하여 문자 메시지 전송
        SmsRequestDTO smsRequestDTO = new SmsRequestDTO();
        smsRequestDTO.setReservationId(reservationId);

//        smsService.sendSms(smsRequestDTO);  // 문자 메시지 전송
        // 문자 메시지 전송은 DB 저장 후, 별도로 호출
        sendSmsAsync(smsRequestDTO);  // 비동기로 처리

        // 성공적으로 저장되면 PhotoResponseDTO를 반환
        return new ResponseEntity<>(response, HttpStatus.CREATED);
    }

    // 문자 메시지 전송을 비동기로 처리
    private void sendSmsAsync(SmsRequestDTO smsRequestDTO) {
        // 문자 메시지 전송을 별도로 실행하여 트랜잭션에 영향을 미치지 않도록 처리
        new Thread(() -> {
            try {
                smsService.sendSms(smsRequestDTO);  // 문자 메시지 전송
            } catch (Exception e) {
                // 예외 처리 로직 추가 (예: 로그 출력)
                e.printStackTrace();
            }
        }).start();
    }

}
