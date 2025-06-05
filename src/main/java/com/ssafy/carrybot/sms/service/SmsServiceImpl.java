package com.ssafy.carrybot.sms.service;

import com.ssafy.carrybot.config.SmsConfig;
import com.ssafy.carrybot.photo.entity.Photo;
import com.ssafy.carrybot.photo.repository.PhotoRepository;
import com.ssafy.carrybot.reservation.repository.ReservationRepository;
import com.ssafy.carrybot.sms.dto.request.SmsRequestDTO;
import com.ssafy.carrybot.sms.dto.response.SmsResponseDTO;
import com.ssafy.carrybot.sms.entity.Sms;
import com.ssafy.carrybot.sms.repository.SmsRepository;
import net.nurigo.sdk.NurigoApp;
import net.nurigo.sdk.message.model.Message;
import net.nurigo.sdk.message.request.SingleMessageSendingRequest;
import net.nurigo.sdk.message.service.DefaultMessageService;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.stereotype.Service;

@Service
public class SmsServiceImpl implements SmsService {

    private final SmsConfig smsConfig;
    private final DefaultMessageService messageService;
    private final SmsRepository smsRepository;

    private final PhotoRepository photoRepository;  // PhotoRepository 추가
    private final ReservationRepository reservationRepository;  // ReservationRepository 추가

    private final BitlyService bitlyService;  // BitlyService 추가

    @Autowired
    public SmsServiceImpl(SmsConfig smsConfig, SmsRepository smsRepository, PhotoRepository photoRepository, ReservationRepository reservationRepository, BitlyService bitlyService) {
        this.smsConfig = smsConfig;
        this.smsRepository = smsRepository;

        this.photoRepository = photoRepository;
        this.reservationRepository = reservationRepository;
        this.bitlyService = bitlyService;
        this.messageService = NurigoApp.INSTANCE.initialize(smsConfig.getApiKey(), smsConfig.getApiSecret(), "https://api.coolsms.co.kr");

    }

    @Override
    public void sendSms(SmsRequestDTO smsRequestDTO) {

        // reservationId로 Reservation을 찾고 해당 Reservation에 관련된 Photo를 찾음
        Long reservationId = smsRequestDTO.getReservationId();
        Photo photo = photoRepository.findByReservationId(reservationId)  // 예약 ID로 사진 조회
                .orElseThrow(() -> new RuntimeException("No photo found for this reservation"));

        // 해당 예약에 대한 사진 URL을 가져옴
        String photoUrl = photo.getImageUrl();  // S3 URL 가져오기

        // Reservation에서 전화번호를 가져옴
        String phoneNumber = reservationRepository.findById(reservationId)
                .orElseThrow(() -> new RuntimeException("Reservation not found"))
                .getPhoneNumber();  // 예약에서 전화번호 추출

        // BitlyService로 URL 단축
        String shortUrl = bitlyService.shortenUrl(photoUrl);  // S3 URL을 단축된 URL로 변경

        // 문자 메시지 내용에 사진 URL을 추가
        String messageContent = "[짐돌이] 승객님의 짐을 배달했어요! 사진을 확인해 보세요. " + shortUrl;  // 메시지 내용에 URL 추가

        // Coolsms에서 제공하는 기본 Message 객체 사용
        Message message = new Message();
        message.setFrom(smsConfig.getSenderNumber());
        message.setTo(phoneNumber);
        message.setText(messageContent);

        try {
            // 기본 Message 객체 사용하여 메시지 전송
            SingleMessageSendingRequest request = new SingleMessageSendingRequest(message);
            messageService.sendOne(request); // 그대로 사용

            // 문자 메시지 전송 후, SMS 객체를 Delivery와 연결하여 저장
            Sms sms = Sms.builder()
                    .phoneNumber(phoneNumber)
                    .messageContent(messageContent)
                    .status("SENT")
                    .build();
            smsRepository.save(sms);
        } catch (Exception e) {
            Sms sms = Sms.builder()
                    .phoneNumber(phoneNumber)
                    .messageContent(messageContent)
                    .status("FAILED")
                    .build();
            smsRepository.save(sms);
            throw new RuntimeException("SMS 전송 실패: " + e.getMessage());
        }
    }

    @Override
    public SmsResponseDTO getMessageList() {
        // 여기에 Coolsms에서 메시지 목록을 가져오는 로직을 추가합니다.
        return new SmsResponseDTO("SUCCESS", "메시지 목록 조회 완료");
    }

    @Override
    public SmsResponseDTO getBalance() {
        // 잔액 조회 로직 추가
        return new SmsResponseDTO("SUCCESS", "잔액 조회 완료");
    }
}
