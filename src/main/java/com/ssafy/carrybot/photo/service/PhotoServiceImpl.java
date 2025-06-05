package com.ssafy.carrybot.photo.service;

import com.ssafy.carrybot.reservation.repository.ReservationRepository;
import com.ssafy.carrybot.photo.dto.response.PhotoResponseDTO;
import com.ssafy.carrybot.photo.entity.Photo;
import com.ssafy.carrybot.photo.repository.PhotoRepository;
import com.ssafy.carrybot.reservation.entity.Reservation;
import jakarta.persistence.EntityNotFoundException;
import org.springframework.stereotype.Service;
import lombok.RequiredArgsConstructor;
import org.springframework.transaction.annotation.Transactional;

@Service
@RequiredArgsConstructor
public class PhotoServiceImpl implements PhotoService {
    private final PhotoRepository photoRepository;
    private final S3Service s3Service;  // S3Service 의존성 주입
    private final ReservationRepository reservationRepository;  // 예약 정보 리포지토리

    // S3 URL을 받아서 DB에 저장하고 DTO로 반환하는 메서드
    @Transactional
    @Override
    public PhotoResponseDTO savePhotoToS3AndDb(String imageBase64, Long reservationId) {
        // S3에 이미지를 업로드하고 URL을 반환
        String s3Url = s3Service.uploadImage(imageBase64); // 이미지를 S3에 업로드하고 URL을 반환
        System.out.println("S3 URL: " + s3Url);  // S3 URL을 로그로 확

        // Reservation 객체 찾기
        Reservation reservation = reservationRepository.findById(reservationId)
                .orElseThrow(() -> new EntityNotFoundException("Reservation not found"));

        // S3 URL을 DB에 저장하는 로직
        Photo photo = Photo.builder()
                .reservation(reservation)  // Reservation 객체와 연결
                .imageUrl(s3Url)
                .build();

        System.out.println("Saving photo to DB with imageUrl: " + s3Url);  // DB 저장 전에 로그 추가

        // DB에 저장
        Photo savedPhoto = photoRepository.save(photo);

        System.out.println("Saved photo ID: " + savedPhoto.getId());  // DB 저장 후 로그

        // Photo를 저장하고, DTO로 변환하여 반환
        return new PhotoResponseDTO(savedPhoto.getId(), savedPhoto.getImageUrl());
    }

    // `createPhoto` 메서드 구현
    @Override
    public PhotoResponseDTO createPhoto(String imageUrl) {
        // DB에 이미지를 저장하는 로직 (예시로 URL만 저장)
        Photo photo = Photo.builder()
                .imageUrl(imageUrl)
                .build();

        // DB에 저장
        Photo savedPhoto = photoRepository.save(photo);

        // 저장된 데이터 반환
        return new PhotoResponseDTO(savedPhoto.getId(), savedPhoto.getImageUrl());
    }
}
