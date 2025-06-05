package com.ssafy.carrybot.sms.service;

import org.springframework.beans.factory.annotation.Value;
import org.springframework.http.*;
import org.springframework.stereotype.Service;
import org.springframework.web.client.RestClientException;
import org.springframework.web.client.RestTemplate;

@Service
public class BitlyService {

    @Value("${bitly.api.token}")
    private String accessToken;

    // Bitly API를 호출하여 URL을 단축하는 메소드
    public String shortenUrl(String longUrl) {
        System.out.println("🔐 Bitly Access Token: " + accessToken);
        String apiUrl = "https://api-ssl.bitly.com/v4/shorten";

        // Bitly API에 보낼 요청 데이터
        String requestBody = "{\"long_url\": \"" + longUrl + "\"}";

        // 요청 헤더 설정
        HttpHeaders headers = new HttpHeaders();
        headers.setContentType(MediaType.APPLICATION_JSON);
        headers.set("Authorization", "Bearer " + accessToken);

        HttpEntity<String> entity = new HttpEntity<>(requestBody, headers);

        try {
            RestTemplate restTemplate = new RestTemplate();
            ResponseEntity<String> response = restTemplate.exchange(apiUrl, HttpMethod.POST, entity, String.class);

            if (response.getStatusCode() == HttpStatus.OK && response.getBody() != null) {
                String responseBody = response.getBody();
                String shortUrl = responseBody.split("\"link\":\"")[1].split("\"")[0]; // 간단한 파싱
                return shortUrl;
            } else {
                System.err.println("⚠️ Bitly 응답 이상: " + response.getStatusCode());
            }
        } catch (RestClientException e) {
            System.err.println("❌ Bitly shorten 실패: " + e.getMessage());
        } catch (Exception e) {
            System.err.println("❌ 예상치 못한 오류: " + e.getMessage());
        }

        // 실패 시 원본 URL 그대로 반환 (fallback)
        return longUrl;
    }
}
