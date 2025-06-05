package com.ssafy.carrybot.admin.service;


import com.ssafy.carrybot.admin.dto.request.AdminVerifyRequestDto;
import com.ssafy.carrybot.admin.dto.response.AdminVerifyResponseDTO;

import java.util.List;

public interface AdminService {
    AdminVerifyResponseDTO verify(AdminVerifyRequestDto dto);
}