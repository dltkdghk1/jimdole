package com.ssafy.carrybot.reservation.entity;

import com.ssafy.carrybot.flight.entity.Flight;
import jakarta.persistence.*;
import lombok.*;

import java.time.LocalDateTime;

@Entity
@Getter
@NoArgsConstructor(access = AccessLevel.PROTECTED)
@AllArgsConstructor
@Builder
public class Reservation {

    @Id
    @GeneratedValue(strategy = GenerationType.IDENTITY)
    private Long id;

    @Column(name = "reservation_code", unique = true, nullable = false)
    private String reservationCode;

    @Column(name = "phone_number", nullable = false)
    private String phoneNumber;

    @Column(name = "is_expired", nullable = false)
    private boolean isExpired = false;

    @ManyToOne(fetch = FetchType.LAZY)
    @JoinColumn(name = "flight_id", nullable = false)
    private Flight flight;

    public void expire() {
        this.isExpired = true;
    }
}
