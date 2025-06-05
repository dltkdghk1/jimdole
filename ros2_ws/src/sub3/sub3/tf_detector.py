#!/ C:\Python37\python.exe
import numpy as np
import cv2
import rclpy
import os
from rclpy.node import Node
import time
from sensor_msgs.msg import CompressedImage, LaserScan
from ssafy_msgs.msg import BBox

import tensorflow as tf

from sub2.ex_calib import *

from object_detection.utils import label_map_util
from object_detection.utils import visualization_utils as vis_util

params_lidar = {
    "Range" : 90, #min & max range of lidar azimuths
    "CHANNEL" : int(1), #verticla channel of a lidar
    "localIP": "127.0.0.1",
    "localPort": 9094,
    "Block_SIZE": int(1206),
    "X": 0, # meter
    "Y": 0,
    "Z": 0.6,
    "YAW": 0, # deg
    "PITCH": 0,
    "ROLL": 0
}


params_cam = {
    "WIDTH": 320, # image width
    "HEIGHT": 240, # image height
    "FOV": 60, # Field of view
    "localIP": "127.0.0.1",
    "localPort": 1232,
    "Block_SIZE": int(65000),
    "X": 0, # meter
    "Y": 0,
    "Z": 1,
    "YAW": 0, # deg
    "PITCH": 0,
    "ROLL": 0
}


class detection_net_class():
    def __init__(self, sess, graph, category_index):
        
        #session and dir
        self.sess = sess
        self.detection_graph = graph
        self.category_index = category_index

        #init tensor
        self.image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')
        self.boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')
        self.scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
        self.classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
        self.num_detections = \
             self.detection_graph.get_tensor_by_name('num_detections:0')

    def inference(self, image_np):
        image_np_expanded = np.expand_dims(image_np, axis=0)
        
        t_start = time.time()
        (boxes, scores, classes, num_detections) = self.sess.run([self.boxes,
                                                                self.scores,
                                                                self.classes,
                                                                self.num_detections],
        feed_dict={self.image_tensor: image_np_expanded})
        
        image_process = np.copy(image_np)

        idx_detect = np.arange(scores.shape[1]).reshape(scores.shape)[np.where(scores>0.5)]

        boxes_detect = boxes[0, idx_detect, :]

        classes_pick = classes[:, idx_detect]

        vis_util.visualize_boxes_and_labels_on_image_array(image_process,
                np.squeeze(boxes),
                np.squeeze(classes).astype(np.int32),
                np.squeeze(scores),
                self.category_index,
                use_normalized_coordinates=True,
                min_score_thresh=0.5,
                line_thickness=8)
                
        infer_time = time.time()-t_start

        return image_process, infer_time, boxes_detect, scores, classes_pick


def visualize_images(image_out, t_cost):

    font = cv2.FONT_HERSHEY_SIMPLEX
    
    cv2.putText(image_out,'SSD',(30,50), font, 1,(0,255,0), 2, 0)

    cv2.putText(image_out,'{:.4f}s'.format(t_cost),(30,150), font, 1,(0,255,0), 2, 0)
    
    winname = 'Vehicle Detection'
    cv2.imshow(winname, cv2.resize(image_out, (2*image_out.shape[1], 2*image_out.shape[0])))
    cv2.waitKey(1)

     
def img_callback(msg):

    global img_bgr

    np_arr = np.frombuffer(msg.data, np.uint8)
    img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)


def scan_callback(msg):

    global xyz

    R = np.array(msg.ranges)

    x = R*np.cos(np.linspace(0, 2*np.pi, 360))
    y = R*np.sin(np.linspace(0, 2*np.pi, 360))
    z = np.zeros_like(x)

    xyz = np.concatenate([
        x.reshape([-1, 1]),
        y.reshape([-1, 1]),
        z.reshape([-1, 1])
    ], axis=1)
   

def main(args=None):
       
    CWD_PATH = os.getcwd()

    MODEL_NAME = 'ssd_mobilenet_v1_coco_2018_01_28'

    PATH_TO_WEIGHT = os.path.join(CWD_PATH, 'model_weights', \
        MODEL_NAME, 'frozen_inference_graph.pb')

    print(PATH_TO_WEIGHT)
    PATH_TO_LABELS = os.path.join(CWD_PATH, 'model_weights', \
        'data', 'mscoco_label_map.pbtxt')

    NUM_CLASSES = 90

    # Loading label map
    label_map = label_map_util.load_labelmap(PATH_TO_LABELS)
    categories = label_map_util.convert_label_map_to_categories(label_map,
                                                            max_num_classes=NUM_CLASSES,
                                                            use_display_name=True)
    
    category_index = label_map_util.create_category_index(categories)

    detection_graph = tf.Graph()
    with detection_graph.as_default():
        od_graph_def = tf.GraphDef()
        with tf.gfile.GFile(PATH_TO_WEIGHT, "rb") as fid:
            serialized_graph = fid.read()
            od_graph_def.ParseFromString(serialized_graph)
            tf.import_graph_def(od_graph_def, name="")

    # config = tf.ConfigProto()
    config = tf.ConfigProto(device_count={'GPU': 1})
    config.gpu_options.per_process_gpu_memory_fraction = 0.3
    config.gpu_options.allow_growth = True
    config.gpu_options.allocator_type = 'BFC'

    sess2 = tf.Session(graph=detection_graph, config=config)

    ssd_net = detection_net_class(sess2, detection_graph, category_index)

    global g_node

    rclpy.init(args=args)

    g_node = rclpy.create_node('tf_detector')

    subscription_img = g_node.create_subscription(CompressedImage, '/image_jpeg/compressed', img_callback, 3)

    subscription_scan = g_node.create_subscription(LaserScan, '/scan', scan_callback, 3)

    # subscription_scan

    # subscription_img
    

    l2c_trans = LIDAR2CAMTransform(params_cam, params_lidar)

    iter_step = 0

    while rclpy.ok():

        time.sleep(0.05)

        for _ in range(2):

            rclpy.spin_once(g_node)
        
        image_process, infer_time, boxes_detect, scores, classes_pick = ssd_net.inference(img_bgr)

        # 로직 : lidar 좌표 변환 및 정사영

        xyz_p = xyz[np.where(xyz[:, 0]>=0)]

        xyz_c = l2c_trans.transform_lidar2cam(xyz_p)

        xy_i = l2c_trans.project_pts2img(xyz_c, False)

        xyii = np.concatenate([xy_i, xyz_p], axis=1)

        if len(boxes_detect) != 0:

            ih = img_bgr.shape[0]
            iw = img_bgr.shape[1]

            boxes_np = np.array(boxes_detect)

            x = boxes_np[:, 0]*iw
            y = boxes_np[:, 1]*ih
            w = boxes_np[:, 2]*iw - x
            h = boxes_np[:, 3]*ih - y

            bbox = np.vstack([
                x.astype(np.int32).tolist(),
                y.astype(np.int32).tolist(),
                w.astype(np.int32).tolist(),
                h.astype(np.int32).tolist()
            ]).T



            ostate_list = []

            for i in range(bbox.shape[0]):

                x = int(bbox[i, 0])
                y = int(bbox[i, 1])
                w = int(bbox[i, 2])
                h = int(bbox[i, 3])

                print(x, y, x+w, y+h)

                print(xyii)

                cx = x + int(w/2)
                cy = y + int(h/2)
                
                xyv = xyii[np.logical_and(xyii[:, 0]>=cx-0.5*w, xyii[:, 0]<cx+0.5*w), :]
                xyv = xyv[np.logical_and(xyv[:, 1]>=cy-0.5*h, xyv[:, 1]<cy+0.5*h), :]

                ostate = np.mean(xyv[:, 2:], axis=0)

                ostate_list.append(ostate)

            image_process = draw_pts_img(image_process, xy_i[:, 0].astype(np.int32),
                                            xy_i[:, 1].astype(np.int32))

            print(ostate_list)

        visualize_images(image_process, infer_time)

    g_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':

    main()

    