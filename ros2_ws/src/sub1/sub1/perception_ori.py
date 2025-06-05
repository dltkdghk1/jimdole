#!/ C:\Python37\python.exe

import numpy as np
import cv2
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import CompressedImage

# image parser 노드는 이미지를 받아서 opencv 의 imshow로 윈도우창에 띄우는 역할을 합니다.
# 이를 resize나 convert를 사용하여 이미지를 원하는대로 바꿔보세요.

# 노드 로직 순서
# 1. image subscriber 생성
# 2. 카메라 콜백함수에서 compressed image 디코딩
# 3. 이미지 색 채널을 gray scale로 컨버팅
# 4. 이미지 resizing
# 5. 이미지 imshow

class IMGParser(Node):

    def __init__(self):
        super().__init__(node_name='image_convertor')

        self.subscription = self.create_subscription(
            CompressedImage,
            '/image_jpeg/compressed',
            self.img_callback,
            10)
        

    def img_callback(self, msg):

        np_arr = np.frombuffer(msg.data, np.uint8)
        img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        img_gray = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)

        img_resize = cv2.resize(img_gray, (300, 300))

        cv2.imshow("resize and gray", img_resize)
        cv2.waitKey(1)


def main(args=None):

    rclpy.init(args=args)

    image_parser = IMGParser()

    rclpy.spin(image_parser)


if __name__ == '__main__':

    main()