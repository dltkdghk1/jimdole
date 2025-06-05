package com.ssafy.carrybot.chat.controller;

import org.springframework.beans.factory.annotation.Value;
import org.springframework.http.HttpHeaders;
import org.springframework.http.MediaType;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.PostMapping;
import org.springframework.web.bind.annotation.RequestBody;
import org.springframework.web.bind.annotation.RequestMapping;
import org.springframework.web.bind.annotation.RestController;
import org.springframework.web.reactive.function.client.WebClient;
import reactor.core.publisher.Mono;

import java.util.Map;

@RestController
@RequestMapping("/chatbot")
public class ChatProxyController {

    private final WebClient webClient;

    public ChatProxyController(WebClient.Builder webClientBuilder,
                               @Value("${perplexity.api.url}") String apiUrl) {
        this.webClient = webClientBuilder
                .baseUrl(apiUrl)
                .defaultHeader(HttpHeaders.CONTENT_TYPE, MediaType.APPLICATION_JSON_VALUE)
                .build();
    }

    @Value("${perplexity.api.key}")
    private String apiKey;

    @PostMapping
    public Mono<ResponseEntity<String>> proxyToPerplexity(@RequestBody Map<String, Object> requestBody) {
        return webClient.post()
                .uri("/chat/completions")
                .header(HttpHeaders.AUTHORIZATION, "Bearer " + apiKey)
                .bodyValue(requestBody)
                .retrieve()
                .toEntity(String.class)
                .onErrorResume(e -> Mono.just(ResponseEntity.internalServerError().body("퍼플렉시티 호출 실패: " + e.getMessage())));
    }
}

