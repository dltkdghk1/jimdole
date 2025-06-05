# mqtt.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import paho.mqtt.client as mqtt
from dotenv import load_dotenv
from pathlib import Path
import os
import uuid  # 고유 ID 생성용

# .env 파일 경로 지정 및 로딩
env_path = Path(__file__).resolve().parent / '.env'
load_dotenv(dotenv_path=env_path)

# 환경변수 로딩
MQTT_BROKER = os.getenv("MQTT_BROKER")
MQTT_PORT = int(os.getenv("MQTT_PORT"))
MQTT_USERNAME = os.getenv("MQTT_USERNAME")
MQTT_PASSWORD = os.getenv("MQTT_PASSWORD")
BASE_MQTT_CLIENT_ID = os.getenv("MQTT_CLIENT_ID")  # 기본 client id

def create_mqtt_client(suffix=""):
    # 클라이언트 ID에 고유 suffix를 붙여 생성
    client_id = BASE_MQTT_CLIENT_ID + ("-" + suffix if suffix else "-" + str(uuid.uuid4()))
    client = mqtt.Client(client_id)
    client.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)
    client.connect(MQTT_BROKER, MQTT_PORT, 60)
    return client

class CmdVelMQTTPublisher(Node):
    def __init__(self):
        super().__init__('imu_mqtt_publisher')
        # /imu 토픽 구독 (메시지 타입: Imu)
        self.subscription = self.create_subscription(
            Imu,
            '/imu',
            self.imu_callback,
            10)
        self.subscription  # unused warning 방지

        # 전역 mqtt_client 사용
        self.client = create_mqtt_client()

        # 1초마다 publish 실행
        self.timer = self.create_timer(1.0, self.publish_timer_callback)
        self.latest_msg = None

    def imu_callback(self, msg):
        self.latest_msg = msg

    def publish_timer_callback(self):
        if self.latest_msg is not None:
            linear = self.latest_msg.linear_acceleration.x
            angular = self.latest_msg.angular_velocity.z
            message = f"{linear},{angular}"
            self.client.publish("turtle/imu", message)
            # print(f"Published: {message}", flush=True)

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelMQTTPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
