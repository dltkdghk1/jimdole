package com.ssafy.carrybot.sms.service;

import com.ssafy.carrybot.sms.dto.request.SmsRequestDTO;
import com.ssafy.carrybot.sms.dto.response.SmsResponseDTO;
import net.nurigo.sdk.message.model.Balance;

public interface SmsService {
    void sendSms(SmsRequestDTO requestDTO);
    SmsResponseDTO getMessageList();
    SmsResponseDTO getBalance();
}
