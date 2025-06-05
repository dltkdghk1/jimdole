package com.ssafy.carrybot.sms.entity;

import jakarta.persistence.*;
import lombok.Builder;
import lombok.Getter;
import lombok.NoArgsConstructor;

@Entity
@Getter
@NoArgsConstructor
public class Sms {
    @Id
    @GeneratedValue(strategy = GenerationType.IDENTITY)
    private Long id;

    private String phoneNumber; // 수신 번호
    private String messageContent; // 메시지 내용
    private String status; // 전송 상태 (SENT, FAILED)

    @Builder
    public Sms(String phoneNumber, String messageContent, String status) {
        this.phoneNumber = phoneNumber;
        this.messageContent = messageContent;
        this.status = status;
    }
}
