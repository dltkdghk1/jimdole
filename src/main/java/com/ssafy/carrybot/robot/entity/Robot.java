package com.ssafy.carrybot.robot.entity;

import com.ssafy.carrybot.robot.enums.RobotStatus;
import jakarta.persistence.*;
import lombok.*;

import java.time.LocalDateTime;

@Setter
@Entity
@Getter
@NoArgsConstructor(access = AccessLevel.PROTECTED)
@AllArgsConstructor
@Builder
public class Robot {

    @Id
    @GeneratedValue(strategy = GenerationType.IDENTITY)
    private Long id;

    @Column(name = "current_x")
    private double currentX;

    @Column(name = "current_y")
    private double currentY;

    @Column(name = "start_x")
    private double startX;

    @Column(name = "start_y")
    private double startY;

    @Column(name = "target_gate")
    private String targetGate; // A, B 게이트

    @Enumerated(EnumType.STRING)
    private RobotStatus status;

    @Column(name = "last_updated")
    private LocalDateTime lastUpdated;

    public void updatePosition(double x, double y) {
        this.currentX = x;
        this.currentY = y;
        this.lastUpdated = LocalDateTime.now();
    }

    public void updateStatus(RobotStatus status) {
        this.status = status;
        this.lastUpdated = LocalDateTime.now();
    }
}
