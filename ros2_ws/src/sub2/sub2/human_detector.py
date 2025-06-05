import cv2
import numpy as np
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import CompressedImage, LaserScan
from std_msgs.msg import Float32MultiArray
from ssafy_msgs.msg import BBox


def non_maximum_supression(bboxes, threshold=0.5):
    
    bboxes = sorted(bboxes, key=lambda detections: detections[3],
            reverse=True)
    new_bboxes=[]
    
    new_bboxes.append(bboxes[0])
    
    bboxes.pop(0)

    for _, bbox in enumerate(bboxes):

        for new_bbox in new_bboxes:

            x1_tl = bbox[0]
            x2_tl = new_bbox[0]
            x1_br = bbox[0] + bbox[2]
            x2_br = new_bbox[0] + new_bbox[2]
            y1_tl = bbox[1]
            y2_tl = new_bbox[1]
            y1_br = bbox[1] + bbox[3]
            y2_br = new_bbox[1] + new_bbox[3]
            
            x_overlap = max(0, min(x1_br, x2_br)-max(x1_tl, x2_tl))
            y_overlap = max(0, min(y1_br, y2_br)-max(y1_tl, y2_tl))
            overlap_area = x_overlap * y_overlap
            
            area_1 = bbox[2] * new_bbox[3]
            area_2 = new_bbox[2] * new_bbox[3]
            
            total_area = area_1 + area_2 - overlap_area

            overlap_area = overlap_area / float(total_area)

            if overlap_area < threshold:
                
                new_bboxes.append(bbox)

    return new_bboxes



class HumanDetector(Node):

    def __init__(self):
        super().__init__(node_name='human_detector')

        self.subs_img = self.create_subscription(
            CompressedImage,
            '/image_jpeg/compressed',
            self.img_callback,
            1)

        self.img_bgr = None
        
        self.timer_period = 0.03

        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.bbox_pub_ = self.create_publisher(BBox, '/bbox', 1)

        self.pedes_detector = cv2.HOGDescriptor()                              
        self.pedes_detector.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

        self.able_to_pub = True

    def img_callback(self, msg):

        np_arr = np.frombuffer(msg.data, np.uint8)

        self.img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
  
    def detect_human(self, img_bgr):
    
        self.bbox_msg = BBox()
    
        img_pre = cv2.cvtColor(self.img_bgr, cv2.COLOR_BGR2GRAY)

        (rects_temp, _) = self.pedes_detector.detectMultiScale(img_pre, winStride=(2, 2), padding=(8, 8), scale=2)

        if len(rects_temp) != 0:

            xl, yl, wl, hl = [], [], [], []
            rects = non_maximum_supression(rects_temp)

            for (x,y,w,h) in rects:

                xl.append(x)
                yl.append(y)
                wl.append(w)
                hl.append(h)

                cv2.rectangle(img_bgr, (x,y),(x+w,y+h),(0,255,255), 2)

            if self.able_to_pub:

                self.bbox_msg.num_bbox = len(rects_temp)

                obj_list = np.zeros((len(rects_temp),), dtype=int).tolist()

                print(obj_list)

                self.bbox_msg.idx_bbox = obj_list

                self.bbox_msg.x = np.array(xl, dtype=int).tolist()
                self.bbox_msg.y = np.array(yl, dtype=int).tolist()
                self.bbox_msg.w = np.array(wl, dtype=int).tolist()
                self.bbox_msg.h = np.array(hl, dtype=int).tolist()

        else:
            # pass
            self.bbox_msg.num_bbox = len(rects_temp)

        cv2.imshow("detection result", img_bgr)
        
        cv2.waitKey(1)

    def timer_callback(self):

        if self.img_bgr is not None:

            self.detect_human(self.img_bgr)

            self.bbox_pub_.publish(self.bbox_msg)

        else:
            pass

def main(args=None):

    rclpy.init(args=args)

    hdetector = HumanDetector()

    rclpy.spin(hdetector)

if __name__ == '__main__':

    main()

