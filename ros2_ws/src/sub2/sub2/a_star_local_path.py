#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from math import sqrt
import numpy as np

class AStarLocalPath(Node):
    def __init__(self):
        super().__init__('a_star_local_path')
        self.local_path_pub = self.create_publisher(Path, 'local_path', 10)
        self.global_path_sub = self.create_subscription(Path, '/global_path', self.path_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.global_path_msg = Path()
        self.odom_msg = None
        self.is_path = False
        self.is_odom = False
        self.local_path_size = 30
        self.timer = self.create_timer(0.05, self.timer_callback)
        
    def odom_callback(self, msg):
        self.is_odom = True
        self.odom_msg = msg
        
    def path_callback(self, msg):
        self.is_path = True
        self.global_path_msg = msg
        
    def timer_callback(self):
        if not (self.is_odom and self.is_path):
            return
        
        # 새 local_path 메시지 생성
        local_path_msg = Path()
        local_path_msg.header.frame_id = '/map'
        
        # 로봇의 현재 위치 획득
        x = self.odom_msg.pose.pose.position.x
        y = self.odom_msg.pose.pose.position.y
        
        # global_path에서 현재 위치에 가장 가까운 waypoint 인덱스를 찾음
        current_waypoint = -1
        min_dis = float('inf')
        for i, waypoint in enumerate(self.global_path_msg.poses):
            distance = sqrt((x - waypoint.pose.position.x)**2 + (y - waypoint.pose.position.y)**2)
            self.get_logger().info(f"[local_path] Waypoint {i}: Pos=({waypoint.pose.position.x:.2f}, {waypoint.pose.position.y:.2f}), distance = {distance:.2f}")
            if distance < min_dis:
                min_dis = distance
                current_waypoint = i
        self.get_logger().info(f"[local_path] Selected waypoint index: {current_waypoint}, min_distance: {min_dis:.2f}")
        
        if current_waypoint == -1:
            return  # waypoint를 찾지 못하면 아무것도 publish하지 않음
        
        # global_path 리스트를 현재 waypoint 기준으로 회전시킴
        poses = self.global_path_msg.poses
        rotated_poses = poses[current_waypoint:] + poses[:current_waypoint]
        
        # 회전된 경로에서 인접 waypoint 간 거리가 큰 불연속 구간을 검출하여, 연속 구간만 선택
        continuous_poses = []
        discontinuity_threshold = 3.0  # 임계값 (필요시 조정)
        continuous_poses.append(rotated_poses[0])
        for i in range(1, len(rotated_poses)):
            prev_pose = rotated_poses[i - 1]
            curr_pose = rotated_poses[i]
            d = sqrt((prev_pose.pose.position.x - curr_pose.pose.position.x)**2 +
                     (prev_pose.pose.position.y - curr_pose.pose.position.y)**2)
            if d > discontinuity_threshold:
                self.get_logger().info(f"[local_path] Discontinuity detected at index {i} (d={d:.2f}). Breaking continuous segment.")
                break
            continuous_poses.append(curr_pose)
        
        # local_path_size 개수만큼 waypoint 선택 (연속 구간이 local_path_size 미만이면 전체 사용)
        local_segment = continuous_poses[:self.local_path_size]
        
        # local_path 메시지 생성
        for pose in local_segment:
            tmp_pose = PoseStamped()
            tmp_pose.pose.position.x = pose.pose.position.x
            tmp_pose.pose.position.y = pose.pose.position.y
            tmp_pose.pose.orientation.w = 1.0
            local_path_msg.poses.append(tmp_pose)
        
        self.local_path_pub.publish(local_path_msg)
        
def main(args=None):
    rclpy.init(args=args)
    node = AStarLocalPath()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
