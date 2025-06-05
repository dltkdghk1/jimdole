package com.ssafy.carrybot.delivery.service;

import com.ssafy.carrybot.delivery.dto.request.DeliveryRequestDTO;
import com.ssafy.carrybot.delivery.dto.response.DeliveryResponseDTO;
import com.ssafy.carrybot.delivery.entity.Delivery;
import com.ssafy.carrybot.reservation.entity.Reservation;
import com.ssafy.carrybot.delivery.repository.DeliveryRepository;
import com.ssafy.carrybot.reservation.repository.ReservationRepository;
import com.ssafy.carrybot.delivery.enums.DeliveryStatus;
import jakarta.persistence.EntityNotFoundException;
import lombok.RequiredArgsConstructor;
import org.springframework.beans.factory.annotation.Value;
import org.springframework.stereotype.Service;

@Service
@RequiredArgsConstructor
public class DeliveryServiceImpl implements DeliveryService{
    private final DeliveryRepository deliveryRepository;
    private final ReservationRepository reservationRepository;

    @Override
    public DeliveryResponseDTO createDelivery(DeliveryRequestDTO dto) {

        Reservation reservation = reservationRepository.findByReservationCode(dto.getReservationCode())
                .orElseThrow(() -> new EntityNotFoundException("예약을 찾을 수 없습니다."));

        // Delivery 객체 생성 및 빌드
        Delivery delivery = Delivery.builder()
                .reservation(reservation)
                .status(DeliveryStatus.IN_PROGRESS) // 기본 상태는 "진행 중"
                .build();

        // DB에 저장
        Delivery saved = deliveryRepository.save(delivery);

        // DTO로 변환하여 반환
        return new DeliveryResponseDTO(
                saved.getId(),
                saved.getReservation().getReservationCode(),
                saved.getStatus().toString()
        );
    }

    // 운송 상태 업데이트
    @Override
    public DeliveryResponseDTO updateDeliveryStatus(Long deliveryId, String eventType) {
        // Delivery 찾기
        Delivery delivery = deliveryRepository.findById(deliveryId)
                .orElseThrow(() -> new EntityNotFoundException("운송 정보를 찾을 수 없습니다."));

        // 배송 상태 업데이트
        DeliveryStatus status = null; // status 초기화

        switch (eventType) {
            case "MOVED":
                status = DeliveryStatus.IN_PROGRESS; // 로봇이 이동 중일 때 배송 상태 "진행 중"
                break;
            case "EMERGENCY_STOP":
                status = DeliveryStatus.IN_PROGRESS; // 긴급 정지
                break;
            case "RESUMED":
                status = DeliveryStatus.IN_PROGRESS; // 재개
                break;
            case "COMPLETED":
                status = DeliveryStatus.COMPLETED; // 배송 완료
                break;
        }

        // 상태 업데이트
        delivery.updateStatus(status);

        // 상태 업데이트 후 DB 저장
        Delivery updated = deliveryRepository.save(delivery);

        // DTO로 변환하여 반환
        return new DeliveryResponseDTO(
                updated.getId(),
                updated.getReservation().getReservationCode(),
                updated.getStatus().toString()
        );
    }
}
