package com.ssafy.carrybot.common.exception;

import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.ExceptionHandler;
import org.springframework.web.bind.annotation.RestControllerAdvice;

import java.time.LocalDateTime;
import java.util.HashMap;
import java.util.Map;
import java.util.Objects;

@RestControllerAdvice
public class GlobalExceptionHandler {

    @ExceptionHandler(CustomException.class)
    public ResponseEntity<?> handleCustomException(CustomException e){
        Map<String, Object> body = new HashMap<>();
        body.put("timestamp", LocalDateTime.now());
        body.put("status", e.getErrorCode().getStatus().value());
        body.put("error", e.getErrorCode().name());
        body.put("message", e.getErrorCode().getMessage());

        return ResponseEntity
                .status(e.getErrorCode().getStatus())
                .body(body);
    }
}
