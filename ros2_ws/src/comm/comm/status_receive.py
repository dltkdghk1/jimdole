import rclpy
from rclpy.node import Node
import paho.mqtt.client as mqtt
import json
from dotenv import load_dotenv
from pathlib import Path
import os

# mqtt.py가 있는 디렉토리의 .env 파일 경로 지정 및 로딩
env_path = Path(__file__).resolve().parent / '.env'
load_dotenv(dotenv_path=env_path)

robot_id = ""
event_type = ""

MQTT_BROKER = os.getenv("MQTT_BROKER")
MQTT_PORT = int(os.getenv("MQTT_PORT"))
MQTT_USERNAME = os.getenv("MQTT_USERNAME")
MQTT_PASSWORD = os.getenv("MQTT_PASSWORD")
MQTT_CLIENT_ID = os.getenv("MQTT_CLIENT_ID")

class StatusInfoSubscriber(Node):
    def __init__(self):
        super().__init__('status_info_subscriber')

        # MQTT 클라이언트 설정
        self.client = mqtt.Client(MQTT_CLIENT_ID)
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message

        self.client.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)
        self.client.connect(MQTT_BROKER, MQTT_PORT, 60)
        self.client.subscribe('robot/status')
        self.client.loop_start()

    def on_connect(self, client, userdata, flags, rc):
        self.get_logger().info("MQTT 연결됨")

    def on_message(self, client, userdata, msg):
        global robot_id, event_type 

        try:
            payload_str = msg.payload.decode()  # ⬅️ 바이트 → 문자열
            data = json.loads(payload_str)      # ⬅️ 문자열 → 딕셔너리

            robot_id = data.get("robot_id")
            event_type = data.get("event_type")

            self.get_logger().info(f"로봇 번호: {robot_id}")
            self.get_logger().info(f"상태 정보: {event_type}")

        except Exception as e:
            self.get_logger().error(f"JSON 파싱 실패: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = StatusInfoSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    node.destroy_node()
if __name__ == "__main__":
    main()