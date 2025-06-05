package com.ssafy.carrybot.config;

import org.springframework.context.annotation.Configuration;
import org.springframework.messaging.simp.config.MessageBrokerRegistry;
import org.springframework.web.socket.config.annotation.EnableWebSocketMessageBroker;
import org.springframework.web.socket.config.annotation.StompEndpointRegistry;
import org.springframework.web.socket.config.annotation.WebSocketMessageBrokerConfigurer;

@Configuration
@EnableWebSocketMessageBroker  // 웹소켓 사용 선언
public class WebSocketConfig implements WebSocketMessageBrokerConfigurer {

    @Override
    public void configureMessageBroker(MessageBrokerRegistry config) {
        config.enableSimpleBroker("/topic"); // 메시지 구독용
        config.setApplicationDestinationPrefixes("/app"); // 클라이언트 → 서버 전송용
    }

    @Override
    public void registerStompEndpoints(StompEndpointRegistry registry) {
        // 웹소켓 연결 주소
        registry.addEndpoint("/ws").setAllowedOriginPatterns("*").withSockJS();
    }
}
//ws는 프론트가 연결할 주소
//topic/robot/status는 프론트가 구독(subscribe)할 주소