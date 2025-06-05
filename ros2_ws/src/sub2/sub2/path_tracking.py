#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from ssafy_msgs.msg import TurtlebotStatus
from squaternion import Quaternion
from nav_msgs.msg import Odometry, Path
from math import pi, cos, sin, sqrt, atan2
import numpy as np

class followTheCarrot(Node):
    def __init__(self):
        super().__init__('path_tracking')
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.status_sub = self.create_subscription(TurtlebotStatus, '/turtlebot_status', self.status_callback, 10)
        self.path_sub = self.create_subscription(Path, '/local_path', self.path_callback, 10)

        # 타이머 주기 (50ms)
        self.timer = self.create_timer(0.05, self.timer_callback)

        self.is_odom = False
        self.is_path = False
        self.is_status = False

        self.odom_msg = Odometry()
        self.robot_yaw = 0.0
        self.path_msg = Path()
        self.cmd_msg = Twist()

        # Lookahead Distance 및 선/각속도 관련 파라미터
        self.lfd = 0.1
        self.min_lfd = 0.1
        self.max_lfd = 1.5  # lookahead 거리를 약간 늘림

        # 선속도 및 각속도 제한
        self.max_linear_vel = 1.0
        self.min_linear_vel = 0.2
        self.max_angular_vel = 0.5  # rad/s
        self.angular_gain = 0.5     # 각오차에 대한 비례 gain

        self.dest_threshold = 0.2  # 목적지 도달 임계 거리 (m)

    def timer_callback(self):
        if self.is_status and self.is_odom and self.is_path:
            # 목적지 도착 체크: 경로의 마지막 waypoint와의 거리를 확인
            if len(self.path_msg.poses) > 0:
                dest_pose = self.path_msg.poses[-1].pose
                robot_pose_x = self.odom_msg.pose.pose.position.x
                robot_pose_y = self.odom_msg.pose.pose.position.y
                dest_distance = sqrt((robot_pose_x - dest_pose.position.x)**2 +
                                     (robot_pose_y - dest_pose.position.y)**2)
                if dest_distance < self.dest_threshold:
                    self.get_logger().info("Destination reached. Stopping robot.")
                    self.cmd_msg.linear.x = 0.0
                    self.cmd_msg.angular.z = 0.0
                    self.cmd_pub.publish(self.cmd_msg)
                    return

            if len(self.path_msg.poses) > 1:
                # 현재 로봇의 위치
                robot_pose_x = self.odom_msg.pose.pose.position.x
                robot_pose_y = self.odom_msg.pose.pose.position.y

                # Lookahead Distance 계산 (기존 방식 유지)
                lateral_error = sqrt((self.path_msg.poses[0].pose.position.x - robot_pose_x)**2 +
                                     (self.path_msg.poses[0].pose.position.y - robot_pose_y)**2)
                self.lfd = min(max((lateral_error + 0.1) * 1.5, self.min_lfd), self.max_lfd)
                self.get_logger().info(f"Robot Pos: ({robot_pose_x:.2f}, {robot_pose_y:.2f}), LFD: {self.lfd:.2f}")

                # 목표 path에서 로봇으로부터 lfd에 가까운 포인트 찾기
                min_dis = float('inf')
                forward_point = None
                for waypoint in self.path_msg.poses:
                    d = sqrt((robot_pose_x - waypoint.pose.position.x)**2 +
                             (robot_pose_y - waypoint.pose.position.y)**2)
                    if abs(d - self.lfd) < min_dis:
                        min_dis = abs(d - self.lfd)
                        forward_point = waypoint.pose.position

                if forward_point is not None:
                    # 로봇 좌표계에서 forward point 위치 구하기
                    global_forward_point = [forward_point.x, forward_point.y, 1]
                    trans_matrix = np.array([
                        [cos(self.robot_yaw), -sin(self.robot_yaw), robot_pose_x],
                        [sin(self.robot_yaw),  cos(self.robot_yaw), robot_pose_y],
                        [0, 0, 1]
                    ])
                    inv_trans_matrix = np.linalg.inv(trans_matrix)
                    local_forward_point = inv_trans_matrix.dot(global_forward_point)
                    theta_error = -atan2(local_forward_point[1], local_forward_point[0])
                    self.get_logger().info(f"Local forward point: {local_forward_point}, Theta error: {theta_error:.2f} rad")

                    # 선속도: 각 오차에 따라 선속도를 조절 (오차가 클 경우 속도를 줄여서 회전 보정)
                    error_deg = abs(theta_error * 180 / pi)
                    if error_deg < 10:
                        lin_vel = self.max_linear_vel
                    elif error_deg < 30:
                        lin_vel = self.max_linear_vel * 0.6
                    else:
                        lin_vel = self.min_linear_vel

                    # 각속도: 비례 제어로 계산하고 최대 각속도로 제한
                    ang_vel = self.angular_gain * theta_error
                    if ang_vel > self.max_angular_vel:
                        ang_vel = self.max_angular_vel
                    if ang_vel < -self.max_angular_vel:
                        ang_vel = -self.max_angular_vel

                    self.cmd_msg.linear.x = lin_vel
                    self.cmd_msg.angular.z = ang_vel

                    self.get_logger().info(f"Cmd: linear x = {lin_vel:.2f}, angular z = {ang_vel:.2f}")
                else:
                    self.get_logger().info("No valid forward point found; stopping.")
                    self.cmd_msg.linear.x = 0.0
                    self.cmd_msg.angular.z = 0.0
            else:
                self.get_logger().info("Path message does not contain enough poses; stopping.")
                self.cmd_msg.linear.x = 0.0
                self.cmd_msg.angular.z = 0.0

            self.cmd_pub.publish(self.cmd_msg)

    def odom_callback(self, msg):
        self.is_odom = True
        self.odom_msg = msg
        q = Quaternion(msg.pose.pose.orientation.w,
                       msg.pose.pose.orientation.x,
                       msg.pose.pose.orientation.y,
                       msg.pose.pose.orientation.z)
        # Euler 변환 중 yaw 추출
        _, _, self.robot_yaw = q.to_euler()

    def path_callback(self, msg):
        self.is_path = True
        self.path_msg = msg

    def status_callback(self, msg):
        self.is_status = True
        self.status_msg = msg

def main(args=None):
    rclpy.init(args=args)
    node = followTheCarrot()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
