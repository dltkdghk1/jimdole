package com.ssafy.carrybot.photo.service;

import com.ssafy.carrybot.photo.dto.response.PhotoResponseDTO;
import com.ssafy.carrybot.delivery.entity.Delivery;

public interface PhotoService {
    // 사진 url을 AWS S3에 저장하고 DB에 저장
    PhotoResponseDTO savePhotoToS3AndDb(String imageUrl, Long reservationId);

    // Impl
    PhotoResponseDTO createPhoto(String imageUrl);
}
