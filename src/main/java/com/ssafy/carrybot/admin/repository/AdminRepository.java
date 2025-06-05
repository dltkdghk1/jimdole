package com.ssafy.carrybot.admin.repository;

import com.ssafy.carrybot.admin.entity.Admin;
import org.springframework.data.jpa.repository.JpaRepository;

import java.util.Optional;

public interface AdminRepository extends JpaRepository<Admin, Long> {

    Optional<Admin> findById(Long id);

}