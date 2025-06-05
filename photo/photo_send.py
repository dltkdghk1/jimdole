import base64
import json
import os
import paho.mqtt.client as mqtt
from dotenv import load_dotenv

# .env 파일 로딩
load_dotenv()

# 환경변수 로딩
MQTT_BROKER = os.getenv("MQTT_BROKER").replace("tcp://", "")
MQTT_PORT = 1883
MQTT_USERNAME = os.getenv("MQTT_USERNAME")
MQTT_PASSWORD = os.getenv("MQTT_PASSWORD")
MQTT_CLIENT_ID = os.getenv("MQTT_CLIENT_ID")
MQTT_TOPIC = os.getenv("MQTT_TOPIC")

# MQTT 연결
mqtt_client = mqtt.Client(MQTT_CLIENT_ID)
mqtt_client.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)
mqtt_client.connect(MQTT_BROKER, MQTT_PORT, 60)

# 1. 사진 파일 읽기(# 그 사진이 저장된 위치를 알아내서 거기서 불러와주면 됨)
with open("/tmp/photo.jpg", "rb") as f:
    image_data = f.read()

# 2. base64 인코딩
image_base64 = base64.b64encode(image_data).decode('utf-8')

# 3. 예약번호와 함께 payload 구성
payload = json.dumps({
    "reservation_id": "A1234", # 이거도 백에서 받은 데이터로 수정해야함함
    "image_base64": image_base64
})

# 4. RabbitMQ에 전송
mqtt_client.publish('robot/photo', payload)
print("📤 사진 전송 완료!")
