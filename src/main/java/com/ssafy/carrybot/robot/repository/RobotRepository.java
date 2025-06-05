package com.ssafy.carrybot.robot.repository;

import com.ssafy.carrybot.robot.entity.Robot;
import org.springframework.data.jpa.repository.JpaRepository;

public interface RobotRepository extends JpaRepository<Robot, Long> {
}