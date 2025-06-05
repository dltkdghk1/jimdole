package com.ssafy.carrybot.gate.repository;

import com.ssafy.carrybot.gate.entity.Gate;
import org.springframework.data.jpa.repository.JpaRepository;

import java.util.Optional;

//int를 long으로 jpa가 변환해줌
//기본적인 CRUD (Create, Read, Update, Delete) 기능은 만듦
public interface GateRepository extends JpaRepository<Gate, Long> {
    Optional<Gate> findByGateNumber(String gateNumber);
}