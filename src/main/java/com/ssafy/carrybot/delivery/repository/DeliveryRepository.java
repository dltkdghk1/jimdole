package com.ssafy.carrybot.delivery.repository;

import com.ssafy.carrybot.delivery.entity.Delivery;
import org.springframework.data.jpa.repository.JpaRepository;

public interface DeliveryRepository extends JpaRepository<Delivery, Long> {
}
