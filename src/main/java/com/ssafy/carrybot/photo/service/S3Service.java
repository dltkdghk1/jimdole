package com.ssafy.carrybot.photo.service;

import software.amazon.awssdk.services.s3.S3Client;
import software.amazon.awssdk.services.s3.model.PutObjectRequest;
import software.amazon.awssdk.services.s3.model.PutObjectResponse;

import org.springframework.beans.factory.annotation.Value;
import org.springframework.stereotype.Service;
import lombok.RequiredArgsConstructor;

import java.io.ByteArrayInputStream;
import java.io.InputStream;
import java.util.UUID;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

@Service
//@RequiredArgsConstructor
public class S3Service {

    private static final Logger logger = LoggerFactory.getLogger(S3Service.class);

    private final S3Client s3Client;
    private final String bucketName; // 버킷 이름
    private final String region;

    // 생성자 주입
    public S3Service(S3Client s3Client, String s3BucketName, String region) {
        this.s3Client = s3Client;
        this.bucketName = s3BucketName;
        this.region = region;
    }

    public String uploadImage(String imageBase64) {
        try {
            // Base64 문자열을 디코딩하여 바이트 배열로 변환
            byte[] decodedBytes = java.util.Base64.getDecoder().decode(imageBase64);

            // 업로드할 파일 이름 설정 (UUID + timestamp로 생성하여 중복 방지)
            String fileName = UUID.randomUUID().toString() + "-" + System.currentTimeMillis() + ".jpg";

            // S3 버킷에 업로드하는 요청 객체 생성
            PutObjectRequest putObjectRequest = PutObjectRequest.builder()
                    .bucket(bucketName)
                    .key(fileName) // 저장될 객체의 키 (파일 이름)
                    .build();

            // 바이트 배열을 InputStream으로 변환
            InputStream inputStream = new ByteArrayInputStream(decodedBytes);

            // S3에 업로드 수행
            PutObjectResponse response = s3Client.putObject(putObjectRequest,
                    software.amazon.awssdk.core.sync.RequestBody.fromInputStream(inputStream, decodedBytes.length));

            // 업로드된 파일의 S3 URL 반환 (예시로 생성된 URL 형식)
            return "https://" + bucketName + ".s3." + region + ".amazonaws.com/" + fileName;
        } catch (Exception e) {
            logger.error("S3 업로드 실패: ", e);
            throw new RuntimeException("S3 업로드 실패: " + e.getMessage());
        }
    }

}
