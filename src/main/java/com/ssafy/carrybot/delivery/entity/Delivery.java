package com.ssafy.carrybot.delivery.entity;

import com.ssafy.carrybot.photo.entity.Photo;
import com.ssafy.carrybot.delivery.enums.DeliveryStatus;
import com.ssafy.carrybot.reservation.entity.Reservation;

import jakarta.persistence.*;
import lombok.*;

@Entity
@Table(name = "delivery")
@Getter
@NoArgsConstructor
@AllArgsConstructor
@Builder
public class Delivery {

    @Id
    @GeneratedValue(strategy = GenerationType.IDENTITY)
    private Long id;

    @Column(nullable = false)
    @Enumerated(EnumType.STRING)
    private DeliveryStatus status; // 운송 상태 (진행중, 완료 등)

    // 예약정보
    @ManyToOne(fetch = FetchType.LAZY) // Delivery : Reservation (N:1)
    @JoinColumn(name = "reservation_id", nullable = false) // Reservation을 FK로 가져옴
    private Reservation reservation;


    // 배달 상태 업데이트
    public void updateStatus(DeliveryStatus status) {
        this.status = status;
    }


}
