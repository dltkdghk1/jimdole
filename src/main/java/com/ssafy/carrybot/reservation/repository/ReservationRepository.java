package com.ssafy.carrybot.reservation.repository;

import com.ssafy.carrybot.reservation.entity.Reservation;
import org.springframework.data.jpa.repository.EntityGraph;
import org.springframework.data.jpa.repository.JpaRepository;

import java.util.Optional;

public interface ReservationRepository extends JpaRepository<Reservation, Long> {

    Optional<Reservation> findByReservationCode(String reservationCode);

    // 새로 추가된, 연관된 flight + gate를 함께 fetch하는 메서드
    @EntityGraph(attributePaths = {"flight", "flight.gate"})
    Optional<Reservation> findWithFlightAndGateByReservationCodeIgnoreCase(String reservationCode);
}

//JPA가 DB에 직접 접근하는 코드를 자동으로 만들어주는 인터페이스
//reservationCode로 조회하는 api가 필요
//해당 예약이 없을 수도 있어서 null-safe하게 감싸줌