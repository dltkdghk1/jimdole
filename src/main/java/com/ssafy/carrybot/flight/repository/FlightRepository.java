package com.ssafy.carrybot.flight.repository;

import com.ssafy.carrybot.flight.entity.Flight;
import org.springframework.data.jpa.repository.JpaRepository;

public interface FlightRepository extends JpaRepository<Flight, Long> {
}


//JPA에서 DB에 직접 접근하는 역할을 담당
//→ insert, find, delete, update 등을 가능하게 해주는 곳