package com.ssafy.carrybot.sms.repository;

import com.ssafy.carrybot.sms.entity.Sms;
import org.springframework.data.jpa.repository.JpaRepository;

public interface SmsRepository extends JpaRepository<Sms, Long> {
}
