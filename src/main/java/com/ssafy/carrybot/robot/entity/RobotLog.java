package com.ssafy.carrybot.robot.entity;

import com.ssafy.carrybot.robot.enums.EventType;
import jakarta.persistence.*;
import lombok.*;

import java.time.LocalDateTime;


@Entity
@Getter
@NoArgsConstructor(access = AccessLevel.PROTECTED)
@AllArgsConstructor
@Builder
public class RobotLog {

    @Id
    @GeneratedValue(strategy = GenerationType.IDENTITY)
    private Long id;

    @Enumerated(EnumType.STRING)
    private EventType eventType;

    private String event;

    @Column(name = "timestamp", nullable = false)
    private LocalDateTime timestamp;

    private String reservationCode;
    private String gateNumber;

    @Column(name = "from_x")
    private double fromX;

    @Column(name = "from_y")
    private double fromY;

    @Column(name = "to_x")
    private double toX;

    @Column(name = "to_y")
    private double toY;

    @Setter
    @ManyToOne(fetch = FetchType.LAZY)
    @JoinColumn(name = "robot_id")
    private Robot robot;

}

