package com.ssafy.carrybot.delivery.service;

import com.ssafy.carrybot.delivery.dto.request.DeliveryRequestDTO;
import com.ssafy.carrybot.delivery.dto.response.DeliveryResponseDTO;
import com.ssafy.carrybot.delivery.enums.DeliveryStatus;

public interface DeliveryService {

    DeliveryResponseDTO createDelivery(DeliveryRequestDTO dto); // 운송정보 생성
    DeliveryResponseDTO updateDeliveryStatus(Long deliveryId, String eventType); // 운송 enum상태 업데이트
}
