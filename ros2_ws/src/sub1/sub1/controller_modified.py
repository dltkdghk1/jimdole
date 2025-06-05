#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path
from ssafy_msgs.msg import TurtlebotStatus
from math import sqrt, atan2, cos, sin, pi
import numpy as np
from squaternion import Quaternion

class ControllerModified(Node):
    def __init__(self):
        super().__init__('controller_modified')
        # Publisher: 명령을 내보낼 cmd_vel 토픽
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Subscriber: odom, local_path, 터틀봇 상태를 구독
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.path_sub = self.create_subscription(Path, '/local_path', self.path_callback, 10)
        self.status_sub = self.create_subscription(TurtlebotStatus, '/turtlebot_status', self.status_callback, 10)

        self.timer = self.create_timer(0.05, self.timer_callback)

        # 데이터 저장용 변수들
        self.odom_msg = None
        self.path_msg = None
        self.status_msg = None
        self.is_odom = False
        self.is_path = False
        self.is_status = False

        self.cmd_msg = Twist()

        # Lookahead distance 설정 (당근까지의 거리)
        self.lfd = 0.5        # 초기 값 (필요시 동적 조정 가능)
        self.min_lfd = 0.1
        self.max_lfd = 1.0

        self.robot_yaw = 0.0  # 로봇의 현재 heading (Euler angle)

    def odom_callback(self, msg):
        self.odom_msg = msg
        self.is_odom = True
        # Quaternion을 Euler 각도로 변환하여 robot_yaw 추출
        q = Quaternion(msg.pose.pose.orientation.w,
                       msg.pose.pose.orientation.x,
                       msg.pose.pose.orientation.y,
                       msg.pose.pose.orientation.z)
        _, _, self.robot_yaw = q.to_euler()

    def path_callback(self, msg):
        self.path_msg = msg
        self.is_path = True

    def status_callback(self, msg):
        self.status_msg = msg
        self.is_status = True

    def timer_callback(self):
        # 필요한 모든 데이터가 수신된 경우에만 제어 로직 실행
        if self.is_odom and self.is_path and self.is_status:
            robot_x = self.odom_msg.pose.pose.position.x
            robot_y = self.odom_msg.pose.pose.position.y

            # Lookahead distance를 고정 값 또는 로봇 속도에 따라 동적으로 설정할 수 있음
            # 여기서는 고정 0.5 m 사용
            self.lfd = 0.5

            forward_point = None
            min_diff = float('inf')
            # local_path에서 로봇과의 거리가 lookahead distance에 가장 가까운 포인트(당근) 선택
            for pose in self.path_msg.poses:
                dx = pose.pose.position.x - robot_x
                dy = pose.pose.position.y - robot_y
                dist = sqrt(dx**2 + dy**2)
                diff = abs(dist - self.lfd)
                if diff < min_diff:
                    min_diff = diff
                    forward_point = pose.pose.position

            if forward_point is not None:
                self.get_logger().info(f"[controller] Selected Forward Point: ({forward_point.x:.2f}, {forward_point.y:.2f}), min_diff: {min_diff:.2f}")
                # 전역 좌표계상의 forward_point와 현재 로봇 위치를 이용하여 상대 위치 계산
                dx = forward_point.x - robot_x
                dy = forward_point.y - robot_y
                self.get_logger().info(f"[controller] dx: {dx:.2f}, dy: {dy:.2f}")
                # 로봇의 현재 heading(self.robot_yaw)를 고려하여 로컬 좌표계로 변환
                local_x = cos(self.robot_yaw)*dx + sin(self.robot_yaw)*dy
                local_y = -sin(self.robot_yaw)*dx + cos(self.robot_yaw)*dy
                self.get_logger().info(f"[controller] Local Coordinates: ({local_x:.2f}, {local_y:.2f})")
                # 로컬 좌표계에서 당근의 방향(각도) 계산
                theta_error = -atan2(local_y, local_x)

                # 간단한 비례 제어 (gain은 상황에 맞게 조정)
                k_ang = 1.0
                angular_z = k_ang * theta_error

                # 로봇의 헤딩 오차가 작을 때만 선속도 부여
                if abs(theta_error) < 0.2:
                    linear_x = 0.3
                else:
                    linear_x = 0.0

                self.cmd_msg.linear.x = linear_x
                self.cmd_msg.angular.z = angular_z
                self.cmd_pub.publish(self.cmd_msg)

                self.get_logger().info(f"Forward point: ({forward_point.x:.2f}, {forward_point.y:.2f}), theta_error: {theta_error:.2f}")
            else:
                # 당근 포인트가 없으면 정지
                self.cmd_msg.linear.x = 0.0
                self.cmd_msg.angular.z = 0.0
                self.cmd_pub.publish(self.cmd_msg)
                self.get_logger().info("No forward point found in local path.")

def main(args=None):
    rclpy.init(args=args)
    node = ControllerModified()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
