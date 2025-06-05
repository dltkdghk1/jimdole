package com.ssafy.carrybot.delivery.controller;

import com.ssafy.carrybot.delivery.dto.request.DeliveryRequestDTO;
import com.ssafy.carrybot.delivery.dto.response.DeliveryResponseDTO;
import com.ssafy.carrybot.delivery.enums.DeliveryStatus;
import com.ssafy.carrybot.delivery.service.DeliveryService;
import lombok.RequiredArgsConstructor;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.RestController;

import org.springframework.web.bind.annotation.*;

@RestController
@RequiredArgsConstructor
@RequestMapping("/delivery")
public class DeliveryController {

    private final DeliveryService deliveryService;

    @PostMapping
    public ResponseEntity<DeliveryResponseDTO> createDelivery(@RequestBody DeliveryRequestDTO dto) {
        // 운송 생성
        return ResponseEntity.ok(deliveryService.createDelivery(dto));
    }

    @PatchMapping("/{deliveryId}/status")
    public ResponseEntity<DeliveryResponseDTO> updateDeliveryStatus(@PathVariable Long deliveryId,
                                                                    @RequestBody String status) {

        // status 값이 "completed", "COMPLETED", "ComPLeTeD" 등 모두 처리 가능하게 toUpperCase 적용
        DeliveryStatus deliveryStatus = DeliveryStatus.valueOf(status.toUpperCase());

        // 운송 상태 업데이트
        return ResponseEntity.ok(deliveryService.updateDeliveryStatus(deliveryId, status));
    }
}
