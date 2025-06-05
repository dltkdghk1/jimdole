package com.ssafy.carrybot.photo.entity;
import com.ssafy.carrybot.delivery.entity.Delivery;

import com.ssafy.carrybot.reservation.entity.Reservation;
import jakarta.persistence.*;
import lombok.AllArgsConstructor;
import lombok.Builder;
import lombok.Getter;
import lombok.NoArgsConstructor;

@Entity
@Table(name = "photo")
@Getter
@NoArgsConstructor
@AllArgsConstructor
@Builder
public class Photo {

    @Id
    @GeneratedValue(strategy = GenerationType.IDENTITY)
    private Long id;

    @Column(nullable = false)
    private String imageUrl;  // 사진의 URL 저장

    @ManyToOne(fetch = FetchType.LAZY)
    @JoinColumn(name = "reservation_id", nullable = true)  // nullable을 true로 설정
    private Reservation reservation;  // Reservation과의 관계 (nullable=true)

    // String imageUrl만 받는 생성자 추가
    public Photo(String imageUrl) {
        this.imageUrl = imageUrl;
    }
}

