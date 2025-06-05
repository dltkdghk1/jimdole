package com.ssafy.carrybot.sms.controller;

import com.ssafy.carrybot.photo.entity.Photo;
import com.ssafy.carrybot.photo.repository.PhotoRepository;
import com.ssafy.carrybot.reservation.repository.ReservationRepository;  // ReservationRepository import 추가
import com.ssafy.carrybot.reservation.entity.Reservation;  // Reservation 엔티티 import

import com.ssafy.carrybot.sms.dto.request.SmsRequestDTO;
import com.ssafy.carrybot.sms.dto.response.SmsResponseDTO;
import com.ssafy.carrybot.sms.service.SmsService;
import jakarta.persistence.EntityNotFoundException;
import lombok.RequiredArgsConstructor;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.*;

@RestController
@RequiredArgsConstructor
@RequestMapping("/sms")
public class SmsController {

    final SmsService smsService;
    private final ReservationRepository reservationRepository; // DeliveryRepository 주입
    private final PhotoRepository photoRepository;  // PhotoRepository 주입

//    public SmsController() {
//        // 반드시 계정 내 등록된 유효한 API 키, API Secret Key를 입력해주셔야 합니다!
//        this.smsService = NurigoApp.INSTANCE.initialize("SMS_API_KEY", "SMS_API_SECRET", "https://api.coolsms.co.kr");
//    }

    /**
     * 메시지 조회 예제
     */
    @GetMapping("/get-message-list")
    public ResponseEntity<SmsResponseDTO> getMessageList() {
        SmsResponseDTO response = smsService.getMessageList();
        return ResponseEntity.ok(response);
    }
        // 검색 조건이 있는 경우에 MessagListRequest를 초기화 하여 getMessageList 함수에 파라미터로 넣어서 검색할 수 있습니다!.
        // 수신번호와 발신번호는 반드시 -,* 등의 특수문자를 제거한 01012345678 형식으로 입력해주셔야 합니다!

        // 검색할 건 수, 값 미지정 시 20건 조회, 최대 500건 까지 설정 가능
        // request.setLimit(1);

        // 조회 후 다음 페이지로 넘어가려면 조회 당시 마지막의 messageId를 입력해주셔야 합니다!
        // request.setStartKey("메시지 ID");

        // request.setTo("검색할 수신번호");
        // request.setFrom("검색할 발신번호");

        // 메시지 상태 검색, PENDING은 대기 건, SENDING은 발송 중,COMPLETE는 발송완료, FAILED는 발송에 실패한 모든 건입니다.
        /*
        request.setStatus(MessageStatusType.PENDING);
        request.setStatus(MessageStatusType.SENDING);
        request.setStatus(MessageStatusType.COMPLETE);
        request.setStatus(MessageStatusType.FAILED);
        */

        // request.setMessageId("검색할 메시지 ID");

        // 검색할 메시지 목록
        /*
        ArrayList<String> messageIds = new ArrayList<>();
        messageIds.add("검색할 메시지 ID");
        request.setMessageIds(messageIds);
         */

        // 조회 할 메시지 유형 검색, 유형에 대한 값은 아래 내용을 참고해주세요!
        // SMS: 단문
        // LMS: 장문
        // MMS: 사진문자
        // ATA: 알림톡
        // CTA: 친구톡
        // CTI: 이미지 친구톡
        // NSA: 네이버 스마트알림
        // RCS_SMS: RCS 단문
        // RCS_LMS: RCS 장문
        // RCS_MMS: RCS 사진문자
        // RCS_TPL: RCS 템플릿문자
        // request.setType("조회 할 메시지 유형");


    /**
     * 단일 메시지 발송 예제
     */
    @PostMapping("/send-one")
    public ResponseEntity<SmsResponseDTO> sendOne(@RequestBody SmsRequestDTO smsRequestDTO) {

        // reservationId를 받아 Reservation 조회 후 SMS 전송
        Long reservationId = smsRequestDTO.getReservationId();  // reservationId는 SmsRequestDTO에서 받아옴
        Reservation reservation = reservationRepository.findById(reservationId)
                .orElseThrow(() -> new EntityNotFoundException("Reservation not found"));




        // 해당 예약에 대한 사진 URL을 조회
        Photo photo = photoRepository.findByReservationId(reservationId)
                .orElseThrow(() -> new EntityNotFoundException("No photo found for reservation"));

        String photoUrl = photo.getImageUrl();  // 사진 URL 가져오기

        // 메시지 내용에 사진 URL 추가
        String messageContent = smsRequestDTO.getMessageContent() + " " + photoUrl;

        smsService.sendSms(smsRequestDTO);
        SmsResponseDTO response = new SmsResponseDTO("SUCCESS", "SMS 전송이 완료되었습니다.");
        return ResponseEntity.ok(response);
    }

    /**
     * 잔액 조회 예제
     */
    @GetMapping("/get-balance")
    public ResponseEntity<SmsResponseDTO> getBalance() {
        SmsResponseDTO response = smsService.getBalance();
        return ResponseEntity.ok(response);
    }
}
