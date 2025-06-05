package com.ssafy.carrybot.admin.service;

import com.ssafy.carrybot.admin.dto.request.AdminVerifyRequestDto;
import com.ssafy.carrybot.admin.dto.response.AdminVerifyResponseDTO;
import com.ssafy.carrybot.admin.entity.Admin;
import com.ssafy.carrybot.admin.repository.AdminRepository;
import lombok.RequiredArgsConstructor;
import org.springframework.stereotype.Service;

@Service
@RequiredArgsConstructor
public class AdminServiceImpl implements AdminService {

    private final AdminRepository adminRepository;

    @Override
    public AdminVerifyResponseDTO verify(AdminVerifyRequestDto dto) {
        System.out.println("전달받은 비밀번호: [" + dto.getPassword() + "]");

        // 예시: 항상 ID = 1번 관리자 기준으로 로그인 체크
        Admin admin = adminRepository.findById(1L)
                .orElseThrow(() -> new RuntimeException("관리자 정보가 존재하지 않습니다."));

        if (admin.getPassword().equals(dto.getPassword())) {
            return new AdminVerifyResponseDTO("SU", "로그인 성공");
        } else {
            return new AdminVerifyResponseDTO("FA", "비밀번호가 일치하지 않습니다.");
        }
    }


}