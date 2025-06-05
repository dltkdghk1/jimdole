import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import os
import numpy as np


class pathPub(Node):

    def __init__(self):
        super().__init__('path_pub')
        print("Initializing node 'path_pub'.")

        # Publisher 생성
        self.global_path_pub = self.create_publisher(Path, 'global_path', 10)
        self.local_path_pub = self.create_publisher(Path, 'local_path', 10)
        print("Publishers for 'global_path' and 'local_path' have been created.")

        # SLAM 맵 데이터 경로 설정
        pkg_path = os.getcwd()
        back_folder = '..'
        folder_name = 'path'
        slam_map_file = 'map.txt'  # SLAM 맵 데이터 파일 이름
        slam_map_path = os.path.join(pkg_path, back_folder, folder_name, slam_map_file)
        print("SLAM map file path:", slam_map_path)

        if not os.path.exists(slam_map_path):
            print("Error: SLAM map file not found:", slam_map_path)
            return
        else:
            print("SLAM map file found.")

        # SLAM 맵 데이터 읽기
        print("Reading SLAM map file...")
        self.slam_map = self.read_slam_map(slam_map_path)
        print("SLAM map loaded successfully. Shape:", self.slam_map.shape)

        # 경로 생성
        print("Generating global path from SLAM map data...")
        self.global_path_msg = self.generate_path_from_map(self.slam_map)
        print("Global path generated with", len(self.global_path_msg.poses), "poses.")

        # 타이머 설정 (0.02초 주기)
        time_period = 0.02
        self.timer = self.create_timer(time_period, self.timer_callback)
        print("Timer created with period of", time_period, "seconds.")

        self.local_path_size = 20
        self.count = 0

    def read_slam_map(self, file_path):
        print("Opening file:", file_path)
        with open(file_path, 'r') as f:
            map_data = []
            for line in f:
                line = line.strip()
                if not line:
                    continue
                print("Reading line:", line)
                try:
                    numbers = list(map(float, line.split()))
                    print("Parsed numbers:", numbers)
                except ValueError as e:
                    print("Error parsing line:", line, "->", e)
                    continue
                map_data.append(numbers)
        print("Finished reading SLAM map file.")
        return np.array(map_data)

    def generate_path_from_map(self, slam_map):
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        print("Creating path message from SLAM map data...")
        for i, coords in enumerate(slam_map):
            pose = PoseStamped()
            pose.pose.position.x = coords[0]  # x 좌표
            pose.pose.position.y = coords[1]  # y 좌표
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
            print(f"Added pose {i}: x={coords[0]}, y={coords[1]}, orientation.w=1.0")
        print("Finished generating path message.")
        return path_msg

    def timer_callback(self):
        self.global_path_pub.publish(self.global_path_msg)
        self.count += 1


def main(args=None):
    rclpy.init(args=args)
    print("rclpy initialized.")
    node = pathPub()
    print("Node created. Spinning...")
    rclpy.spin(node)
    print("Shutting down node.")
    node.destroy_node()
    rclpy.shutdown()
    print("rclpy shutdown.")


if __name__ == '__main__':
    main()