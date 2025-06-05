#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import json
import math
from dotenv import load_dotenv
from pathlib import Path
import os

from std_msgs.msg import String
from sensor_msgs.msg import Imu  # /imu 토픽에 사용되는 메시지 타입
from . import mqtt  # 같은 패키지 내 mqtt.py 모듈 재사용

# mqtt.py에서 전역으로 생성된 mqtt_client 재사용
mqtt_client = mqtt.mqtt_client

# 환경변수로부터 ROBOT_ID를 가져옴 (없으면 빈 문자열)
robot_id = os.getenv("ROBOT_ID", "")

class StatusSend(Node):
    def __init__(self):
        super().__init__('status_send')
        # /initial_goal 토픽 구독: 게이트 정보(예약코드, x, y, gate 등)를 저장만 함
        self.create_subscription(
            String,
            'initial_goal',
            self.initial_goal_callback,
            10
        )
        self.initial_goal_data = None   # 게이트 정보 저장 변수

        # /imu 토픽 구독: IMU 메시지로부터 선형 가속도 및 각속도 정보를 가져옴
        self.create_subscription(
            Imu,
            '/imu',
            self.imu_callback,
            10
        )
        self.latest_imu = None
        self.computed_event_type = None  # "MOVED" 또는 "STOPPED" 저장

    def initial_goal_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
            # 게이트 정보는 단순히 저장만 함 (publish하지 않음)
            self.initial_goal_data = data
            self.get_logger().info("Received initial_goal: " + str(data))
        except Exception as e:
            self.get_logger().error("Failed to parse initial_goal: " + str(e))

    def imu_callback(self, msg: Imu):
        self.latest_imu = msg
        # IMU 메시지에서 linear_acceleration.x와 angular_velocity.z 값을 가져옴
        linear = msg.linear_acceleration.x
        angular = msg.angular_velocity.z

        # 임계값: 선형 가속도 3.0 이상 또는 각속도 1.0 이상이면 MOVED, 아니면 STOPPED로 판별
        # (필요에 따라 임계값은 조정하세요)
        if abs(linear) > 3.0 or abs(angular) > 1.0:
            new_event_type = "MOVED"
        else:
            new_event_type = "STOPPED"

        if new_event_type != self.computed_event_type:
            self.computed_event_type = new_event_type
            self.get_logger().info("Computed event_type updated: " + self.computed_event_type)
            # event_type이 STOPPED일 때만 즉시 MQTT 전송
            if self.computed_event_type == "STOPPED":
                self.send_status()

    def send_status(self):
        # 오직 computed_event_type이 "STOPPED"일 때만 MQTT 전송하도록 함
        if self.computed_event_type != "STOPPED":
            return

        # 게이트 정보가 있으면 해당 값을 사용, 없으면 기본값 사용
        if self.initial_goal_data is not None:
            reservation_code = self.initial_goal_data.get("reservation_code", "")
            gate_val = self.initial_goal_data.get("gate", "")
            x_val = self.initial_goal_data.get("x", 0)
            y_val = self.initial_goal_data.get("y", 0)
        else:
            reservation_code = ""
            gate_val = ""
            x_val = 0
            y_val = 0

        # payload 구성: 게이트 정보와 함께 터틀봇의 상태(여기서는 "STOPPED")만 반영
        payload = json.dumps({
            "reservation_code": reservation_code,
            "event_type": self.computed_event_type,  # "STOPPED"만 전송됨.
            "robot_id": robot_id,
            "x": x_val,
            "y": y_val,
            "gate": gate_val
        })
        mqtt_client.publish("robot/status", payload)
        self.get_logger().info("MQTT Status sent: " + payload)

def main(args=None):
    rclpy.init(args=args)
    node = StatusSend()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
