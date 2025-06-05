#!/ C:\Python37\python.exe

import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from ssafy_msgs.msg import HandControl
import os
import sys

class IMGParser(Node):
    def __init__(self):
        super().__init__('image_convertor_custom')
        #print("노드 생성 완료")
        
        # /image_jpeg/compressed 토픽 구독: 이미지 데이터를 수신
        self.subscription = self.create_subscription(
            CompressedImage,
            '/image_jpeg/compressed',
            self.img_callback,
            10)
        # print("camera subscribe")
        self.latest_frame = None  # 최신 이미지 프레임 저장 변수

        # /hand_control 토픽 구독 (control_mode 정보)
        self.hand_control_sub = self.create_subscription(
            HandControl,
            '/hand_control',
            self.hand_control_callback,
            10)
        # print("hand_control subscribe")

    def img_callback(self, msg):
        # 이미지 디코딩: bytes → numpy array → BGR 이미지
        np_arr = np.frombuffer(msg.data, np.uint8)
        img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if img_bgr is None:
            print("[WARN] image decoding failed")
        else:
            self.latest_frame = img_bgr  # 최신 프레임 갱신
            cv2.imshow("img_bgr", img_bgr)
            cv2.waitKey(1)
            # print(f"[img_callback] 이미지 수신: shape = {img_bgr.shape}")

    def hand_control_callback(self, msg: HandControl):
        print(f"[hand_control] Received control_mode = {msg.control_mode}")
        # control_mode가 3이면 현재 프레임 캡쳐 후 저장
        if msg.control_mode == 3:
            print("[hand_control] control_mode == 3 detected. Capturing image...")
            if self.latest_frame is not None and self.latest_frame.size > 0:
                self.try_save_image()
            else:
                print("[hand_control] No valid image available to save.")

    def try_save_image(self):
        # 저장 경로: 바탕화면 (C:\Users\SSAFY\Desktop\captured_image.jpg)
        save_path = os.path.join(os.path.expanduser("~"), "Desktop", "captured_image.jpg")
        try:
            print("[DEBUG] latest_frame shape:", self.latest_frame.shape, "dtype:", self.latest_frame.dtype)
        except Exception as e:
            print("[DEBUG] latest_frame fail:", e)
        ret = cv2.imwrite(save_path, self.latest_frame)
        if ret:
            print(f"[✔] image saved: {save_path}")
        else:
            print(f"[!] image save failed: {save_path}")
        self.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

def main(args=None):
    rclpy.init(args=args)
    image_parser = IMGParser()
    try:
        rclpy.spin(image_parser)
    except KeyboardInterrupt:
        print("[INFO] Ctrl+C")
    finally:
        image_parser.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
