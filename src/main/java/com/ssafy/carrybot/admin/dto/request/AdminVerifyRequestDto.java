package com.ssafy.carrybot.admin.dto.request;

import lombok.Getter;
import lombok.NoArgsConstructor;
import lombok.Setter;

@Getter
@Setter
@NoArgsConstructor
public class AdminVerifyRequestDto {
    private String password;
}