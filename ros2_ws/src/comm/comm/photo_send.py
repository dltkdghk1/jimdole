#!/usr/bin/env python3
import base64
import json
import os
import time
from pathlib import Path
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from comm import mqtt  # 절대 import 사용
from rclpy.qos import QoSProfile, QoSDurabilityPolicy

def create_reservation_node():
    class ReservationSubscriber(Node):
        def __init__(self):
            super().__init__('reservation_subscriber')
            self.reservation_code = None
            qos_profile = QoSProfile(depth=1)
            qos_profile.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
            self.create_subscription(String, 'initial_goal', self.callback, qos_profile)

        def callback(self, msg: String):
            try:
                data = json.loads(msg.data)
                self.reservation_code = data.get("reservation_code", "")
                self.get_logger().info("Received reservation_code: " + self.reservation_code)
            except Exception as e:
                self.get_logger().error("Failed to parse /initial_goal: " + str(e))
    return ReservationSubscriber()

def main(args=None):
    rclpy.init(args=args)
    reservation_node = create_reservation_node()
    
    timeout = 20.0  # 최대 10초 대기
    start_time = time.time()
    while reservation_node.reservation_code is None and (time.time() - start_time) < timeout:
        rclpy.spin_once(reservation_node, timeout_sec=0.5)
    
    reservation_code = reservation_node.reservation_code
    if not reservation_code:
        print("No reservation_code received", flush=True)
        reservation_node.destroy_node()
        rclpy.shutdown()
        exit(1)
    
    reservation_node.destroy_node()
    rclpy.shutdown()

    # 고유한 MQTT 클라이언트 생성 (suffix "photo_send")
    mqtt_client = mqtt.create_mqtt_client("photo_send")

    save_path = os.path.join(os.path.expanduser("~"), "Desktop", "captured_image.jpg")
    timeout_image = 30  # 30초 대기
    elapsed = 0
    while not os.path.exists(save_path) and elapsed < timeout_image:
        time.sleep(0.5)
        elapsed += 0.5

    if not os.path.exists(save_path):
        print("No image", flush=True)
        exit(1)

    with open(save_path, "rb") as f:
        image_data = f.read()

    image_base64 = base64.b64encode(image_data).decode("utf-8")

    payload = json.dumps({
        "reservation_code": reservation_code,
        "image_base64": image_base64
    })

    mqtt_client.publish("robot/photo", payload)
    print(f"{reservation_code} image send finish", flush=True)

if __name__ == "__main__":
    main()
