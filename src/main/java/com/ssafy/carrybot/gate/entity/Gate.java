package com.ssafy.carrybot.gate.entity;

import jakarta.persistence.*;
import lombok.*;

import jakarta.persistence.*;
import lombok.*;

@Entity
@Getter
@NoArgsConstructor(access = AccessLevel.PROTECTED)
@AllArgsConstructor
@Builder
public class Gate {

    @Id
    @GeneratedValue(strategy = GenerationType.IDENTITY)
    private Long id;

    @Column(name = "gate_number", unique = true, nullable = false)
    private String gateNumber; // 'A', 'B' 등

    @Column(nullable = false)
    private double x;

    @Column(nullable = false)
    private double y; //소수점까지 받아야해서 int -> double로 변경
}

//의존관계로 인해 gate-flight-reservation순으로 구현해야함.
