package com.ssafy.carrybot.admin.controller;

import com.ssafy.carrybot.admin.dto.request.AdminVerifyRequestDto;
import com.ssafy.carrybot.admin.dto.response.AdminVerifyResponseDTO;
import com.ssafy.carrybot.admin.service.AdminService;
import lombok.RequiredArgsConstructor;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.PostMapping;
import org.springframework.web.bind.annotation.RequestBody;
import org.springframework.web.bind.annotation.RequestMapping;
import org.springframework.web.bind.annotation.RestController;

@RestController
@RequiredArgsConstructor
@RequestMapping("/admin")
public class AdminController {

    private final AdminService adminService;

    @PostMapping("/verify")
    public ResponseEntity<AdminVerifyResponseDTO> login(@RequestBody AdminVerifyRequestDto dto) {
        return ResponseEntity.ok(adminService.verify(dto));
    }
}