package com.ssafy.carrybot.photo.dto.response;

import lombok.AllArgsConstructor;
import lombok.Getter;
import lombok.NoArgsConstructor;

@Getter
@AllArgsConstructor
@NoArgsConstructor
public class PhotoResponseDTO {
    private Long id; // 사진의 ID
    private String imageUrl; // 사진이 저장된 S3 URL
}
