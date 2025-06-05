#!/ C:\Python37\python.exe

import numpy as np
import cv2
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import CompressedImage

# segmentation image object detection node의 전체 로직 순서
# 1. 노드에 필요한 publisher, subscriber, timer 정의
# 2. compressed image 받기
# 3. bgr 이미지의 binarization
# 4. 물체의 contour 찾기
# 5. 물체의 bounding box 좌표 찾기
# 6. 물체의 bounding box 가 그려진 이미지 show

class IMGParser(Node):

    def __init__(self):
        super().__init__(node_name='image_parser')

        self.subscription = self.create_subscription(
            CompressedImage,
            '/image_jpeg/compressed',
            self.img_callback,
            10)

        self.img_bgr = None

        self.timer_period = 0.03

        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        

    def img_callback(self, msg):

        np_arr = np.frombuffer(msg.data, np.uint8)
        self.img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)


    def find_bbox(self):

        lower_wal = (100, 245, 255)
        upper_wal = (110, 255, 255)        
        lower_bp = (100, 210, 235)
        upper_bp = (110, 220, 255)
        lower_rc = (100, 210, 200)
        upper_rc = (110, 220, 220)
        lower_key = (100, 240, 200)
        upper_key = (110, 250, 220)

        self.img_wal = cv2.inRange(self.img_bgr, lower_wal, upper_wal)

        self.img_bp = cv2.inRange(self.img_bgr, lower_bp, upper_bp)

        self.img_rc = cv2.inRange(self.img_bgr, lower_rc, upper_rc)

        self.img_key = cv2.inRange(self.img_bgr, lower_key, upper_key)

        contours_wal, _ = cv2.findContours(self.img_wal, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        contours_bp, _ = cv2.findContours(self.img_bp, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        contours_rc, _ = cv2.findContours(self.img_rc, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        contours_key, _ = cv2.findContours(self.img_key, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        self.find_cnt(contours_wal)
        
        self.find_cnt(contours_bp)
        
        self.find_cnt(contours_rc)
        
        self.find_cnt(contours_key)


    def find_cnt(self, contours):

        for cnt in contours:
    
            x, y, w, h = cv2.boundingRect(cnt)

            cv2.rectangle(self.img_bgr, (x, y), (x + w, y + h), (0, 0, 255), 2)



    def timer_callback(self):

        if self.img_bgr is not None:
            
            self.find_bbox()

            cv2.imshow("seg_results", self.img_bgr)

            cv2.waitKey(1)
        else:
            pass
        

        
def main(args=None):

    rclpy.init(args=args)

    image_parser = IMGParser()

    rclpy.spin(image_parser)


if __name__ == '__main__':

    main()