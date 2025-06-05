package com.ssafy.carrybot.robot.repository;

import com.ssafy.carrybot.robot.entity.RobotLog;
import org.springframework.data.jpa.repository.JpaRepository;
import org.springframework.data.jpa.repository.JpaSpecificationExecutor;

import java.util.List;

public interface RobotLogRepository extends JpaRepository<RobotLog, Long>, JpaSpecificationExecutor<RobotLog> {
    List<RobotLog> findAllByOrderByTimestampDesc(); //모든 로봇 로그를 시간순(최신순)으로 정렬
    List<RobotLog> findByReservationCode(String reservationCode); //예약 번호로 조회
}
