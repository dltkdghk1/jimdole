import rclpy
from rclpy.node import Node
import paho.mqtt.client as mqtt
import json
from dotenv import load_dotenv
import os

load_dotenv()

MQTT_BROKER = os.getenv("MQTT_BROKER")
MQTT_PORT = int(os.getenv("MQTT_PORT"))
MQTT_USERNAME = os.getenv("MQTT_USERNAME")
MQTT_PASSWORD = os.getenv("MQTT_PASSWORD")
MQTT_CLIENT_ID = os.getenv("MQTT_CLIENT_ID")

class GateInfoSubscriber(Node):
    def __init__(self):
        super().__init__('gate_info_subscriber')

        # MQTT 클라이언트 설정
        self.client = mqtt.Client(MQTT_CLIENT_ID)
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message

        self.client.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)
        self.client.connect(MQTT_BROKER, MQTT_PORT, 60)
        self.client.subscribe('robot/gate')
        self.client.loop_start()

    def on_connect(self, client, userdata, flags, rc):
        self.get_logger().info("✅ MQTT 연결됨")

    def on_message(self, client, userdata, msg):
        try:
            payload_str = msg.payload.decode()  # ⬅️ 바이트 → 문자열
            data = json.loads(payload_str)      # ⬅️ 문자열 → 딕셔너리

            reservation_id = data.get("reservation_code")
            gate = data.get("gate")

            self.get_logger().info(f"📥 받은 예약번호: {reservation_id}")
            self.get_logger().info(f"📥 이동할 GATE: {gate}")

            # 여기서 A* 경로 탐색 시작하면 됨
            # self.navigate_to_gate(gate)

        except Exception as e:
            self.get_logger().error(f"❌ JSON 파싱 실패: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = GateInfoSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
