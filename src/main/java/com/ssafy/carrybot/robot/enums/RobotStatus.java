package com.ssafy.carrybot.robot.enums;

public enum RobotStatus {
    MOVING,          // 이동 중
    STOPPED,         // 도착 or 완료 상태
    EMERGENCY_STOP,   // 수동 긴급 중단
    IDLE //원래는 없앴는데 안쓰더라도 있어야함. 없으면 에러 발생해서.
    //idle은 대기중으로 stopped로 대체 가능해서 일단 생략
}
