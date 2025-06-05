package com.ssafy.carrybot.photo.service;

import com.ssafy.carrybot.photo.repository.PhotoRepository;
import com.ssafy.carrybot.photo.entity.Photo;
import com.ssafy.carrybot.reservation.entity.Reservation;
import com.ssafy.carrybot.reservation.repository.ReservationRepository;
import com.ssafy.carrybot.sms.dto.request.SmsRequestDTO;
import com.ssafy.carrybot.sms.service.SmsService;
import org.springframework.stereotype.Service;
import org.json.JSONObject;
import org.springframework.transaction.annotation.Transactional;

@Service
public class MqttPhotoReceiverService {

    private final S3Service s3Service;
    private final PhotoRepository photoRepository;
    private final SmsService smsService;
    private final ReservationRepository reservationRepository;

    public MqttPhotoReceiverService(S3Service s3Service, PhotoRepository photoRepository,
                                    SmsService smsService, ReservationRepository reservationRepository) {
        this.s3Service = s3Service;
        this.photoRepository = photoRepository;
        this.smsService = smsService;
        this.reservationRepository = reservationRepository;
    }

    @Transactional
    public void handlePhoto(String payload) {
        System.out.println("📥 [handlePhoto] 호출됨!");
        System.out.println("📨 수신된 payload: " + payload);

        try {
            String reservationCode = extractReservationId(payload);
            String imageBase64 = extractImageBase64(payload);

            System.out.println("📌 파싱된 예약번호: " + reservationCode);
            System.out.println("🖼️ 이미지 base64 길이: " + (imageBase64 != null ? imageBase64.length() : "null"));

            Reservation reservation = reservationRepository.findByReservationCode(reservationCode)
                    .orElseThrow(() -> new RuntimeException("❌ Reservation not found: " + reservationCode));

            Long reservationId = reservation.getId();
            System.out.println("🆔 예약 ID: " + reservationId);

            String s3Url = s3Service.uploadImage(imageBase64);
            System.out.println("✅ S3 업로드 성공, URL: " + s3Url);

            // 디버깅 로그 추가
            System.out.println("🔍 Reservation 객체: " + reservation);
            System.out.println("🔍 Reservation ID: " + reservation.getId());
            System.out.println("🔍 S3 URL: " + s3Url);

            // Reservation 객체와 연결된 Photo 저장
            Photo photo = Photo.builder()
                    .imageUrl(s3Url)
                    .reservation(reservation)
                    .build();
            photoRepository.save(photo);

            // 저장 후 로그
            System.out.println("✅ Photo DB 저장 완료: ID=" + photo.getId());

            // 문자 발송
            SmsRequestDTO smsRequestDTO = new SmsRequestDTO();
            smsRequestDTO.setReservationId(reservationId);
            smsRequestDTO.setMessageContent(s3Url);

            smsService.sendSms(smsRequestDTO);
            System.out.println("📤 SMS 전송 완료");

        } catch (Exception e) {
            System.err.println("❌ 사진 처리 중 에러: " + e.getMessage());
            e.printStackTrace();
        }
    }

    private String extractReservationId(String payload) {
        JSONObject json = new JSONObject(payload);
        return json.getString("reservation_code");
    }

    private String extractImageBase64(String payload) {
        JSONObject json = new JSONObject(payload);
        return json.getString("image_base64");
    }
}
