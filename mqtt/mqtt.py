import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import paho.mqtt.client as mqtt
from dotenv import load_dotenv
import os

# .env 파일 로딩
load_dotenv()

# 환경변수 로딩
MQTT_BROKER = os.getenv("MQTT_BROKER")
MQTT_PORT = os.getenv("MQTT_PORT")
MQTT_USERNAME = os.getenv("MQTT_USERNAME")
MQTT_PASSWORD = os.getenv("MQTT_PASSWORD")
MQTT_CLIENT_ID = os.getenv("MQTT_CLIENT_ID")

class CmdVelMQTTPublisher(Node):
    def __init__(self):
        super().__init__('imu_mqtt_publisher')
        # /cmd_vel 토픽 구독 (메시지 타입: Twist)
        self.subscription = self.create_subscription(
            Imu,
            '/imu',
            self.imu_callback,
            10)
        self.subscription  # 변수 사용 경고 방지

        # MQTT 클라이언트 생성 및 브로커 연결
        self.client = mqtt.Client(MQTT_CLIENT_ID)
        self.client.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)
        self.client.connect(MQTT_BROKER, MQTT_PORT, 60)

        # ⏱️ 1초(1.0)마다 publish 실행
        self.timer = self.create_timer(1.0, self.publish_timer_callback)

    def imu_callback(self, msg):
        # 최신 메시지만 저장 (즉시 publish 안 함)
        self.latest_msg = msg

    def publish_timer_callback(self):
        if self.latest_msg is not None:
            linear = self.latest_msg.linear_acceleration.x
            angular = self.latest_msg.angular_velocity.z
            message = f"{linear},{angular}"
            self.client.publish("turtle/imu", message)
            self.get_logger().info(f"Published: {message}")
    

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelMQTTPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()