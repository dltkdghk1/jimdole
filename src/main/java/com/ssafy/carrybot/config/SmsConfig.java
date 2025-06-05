package com.ssafy.carrybot.config;

import org.springframework.beans.factory.annotation.Value;
import org.springframework.context.annotation.Configuration;

@Configuration
public class SmsConfig {

    @Value("${coolsms.api.api_key}")
    private String apiKey;

    @Value("${coolsms.api.api_secret}")
    private String apiSecret;

    @Value("${coolsms.sender.number}")
    private String senderNumber;

    public String getApiKey() {
        return apiKey;
    }

    public String getApiSecret() {
        return apiSecret;
    }

    public String getSenderNumber() {
        return senderNumber;
    }
}
