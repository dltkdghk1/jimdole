package com.ssafy.carrybot.photo.repository;

import com.ssafy.carrybot.photo.entity.Photo;
import org.springframework.data.jpa.repository.JpaRepository;
import org.springframework.stereotype.Repository;

import java.util.Optional;

@Repository
public interface PhotoRepository extends JpaRepository<Photo, Long> {
    // URL을 기준으로 Photo를 조회하는 메소드 추가
    Optional<Photo> findByImageUrl(String imageUrl);  // Optional은 null값을 방지하기 위해 사용

    // reservationId를 기준으로 Photo를 조회하는 메소드 추가
    Optional<Photo> findByReservationId(Long reservationId);  // reservationId를 기준으로 찾기
}
