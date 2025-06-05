package com.ssafy.carrybot.robot.dto.mqtt;

import lombok.Getter;

@Getter
public class RobotStatusMessageDTO {
    private int robot_id;
    private String event_type;
    private double x;
    private double y;
    private String reservation_code;
    private String gate; //stopped 받을 때 필요.
}
