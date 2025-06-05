package com.ssafy.carrybot;

import com.ssafy.carrybot.robot.enums.RobotStatus;
import com.ssafy.carrybot.robot.websocket.RobotWebSocketService;
import lombok.RequiredArgsConstructor;
import org.springframework.web.bind.annotation.*;

import java.time.LocalDateTime;

@RestController
@RequiredArgsConstructor
@RequestMapping("/test")
public class TestController {

    private final RobotWebSocketService robotWebSocketService;

    @PostMapping("/robot/status")
    public void sendTestRobotStatus(
            @RequestParam double x,
            @RequestParam double y,
            @RequestParam String gate,
            @RequestParam String reservationCode
    ) {
        robotWebSocketService.sendRobotStatus(
                x,
                y,
                gate,
                RobotStatus.STOPPED, // 혹은 원하는 상태 enum
                LocalDateTime.now(),
                "테스트 메시지입니다.",
                reservationCode
        );
    }
}
