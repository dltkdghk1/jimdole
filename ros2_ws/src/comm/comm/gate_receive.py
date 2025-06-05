import rclpy
from rclpy.node import Node
import paho.mqtt.client as mqtt
import json
from pathlib import Path
import os
from std_msgs.msg import String
from . import mqtt
from rclpy.qos import QoSProfile, QoSDurabilityPolicy

# 고유한 MQTT 클라이언트 생성 (suffix "gate_receive")
mqtt_client = mqtt.create_mqtt_client("gate_receive")

robot_id = ""
reservation_code = ""
gate = ""
x = 0
y = 0

class GateInfoSubscriber(Node):
    def __init__(self):
        super().__init__('gate_info_subscriber')

        # TRANSIENT_LOCAL QoS 프로파일 생성: depth 1, durability TRANSIENT_LOCAL
        qos_profile = QoSProfile(depth=1)
        qos_profile.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

        # "initial_goal" 토픽으로 메시지 publish (auto_goal_publisher가 구독)
        self.goal_pub = self.create_publisher(String, 'initial_goal', qos_profile)

        # 전역 mqtt_client 사용
        self.client = mqtt_client
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message

        self.client.subscribe('robot/gate')
        self.client.loop_start()


    def on_connect(self, client, userdata, flags, rc):
        print(f" MQTT connect {rc}", flush=True)

    def on_message(self, client, userdata, msg):
        global robot_id, reservation_id, gate, x, y, event_type 

        try:
            print(f"Raw payload: {msg.payload}", flush=True)
            payload_str = msg.payload.decode()  # ⬅ 바이트 → 문자열
            data = json.loads(payload_str)      # ⬅ 문자열 → 딕셔너리
            print(f"Parsed data: {data}", flush=True)
            print(f"Keys: {list(data.keys())}", flush=True)

            robot_id = data.get("robot_id")
            reservation_code = data.get("reservation_code")
            gate = data.get("gate")
            x = data.get('x')
            y = data.get('y')

            print(f"robot_id: {robot_id}", flush=True)
            print(f"reservation_code: {reservation_code}", flush=True)
            print(f"GATE: {gate}", flush=True)
            print(f"x: {x}", flush=True)
            print(f"y: {y}", flush=True)

            # # 메시지를 받은 뒤 통신 종료
            # self.client.loop_stop()
            # self.client.disconnect()
            # rclpy.shutdown()

            # MQTT로 받은 데이터를 ROS 토픽 "initial_goal"으로 publish
            goal_msg = String()
            goal_msg.data = payload_str  # 원본 JSON 문자열을 그대로 전달
            self.goal_pub.publish(goal_msg)
            print("Initial goal published from gate information.", flush=True)
        except Exception as e:
            print(f"JSON error: {e}", flush=True)


def main(args=None):
    rclpy.init(args=args)
    node = GateInfoSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

    # # ROS2 spin은 메시지 처리 중에만 필요함
    # # 메시지 받은 후 shutdown 되기 때문에, spin 중단됨
    # try:
    #     rclpy.spin(node)
    # except:
    #     pass  # shutdown 중 예외가 발생할 수 있음

    # node.destroy_node()
if __name__ == "__main__":
    main()